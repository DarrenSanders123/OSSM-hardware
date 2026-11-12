#include "update.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_flash.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "FirmwareUpdateRuntime.h"
#include "constants/LogTags.h"
#include "constants/Version.h"
#include "ossm/Events.h"
#include "ossm/pages/update.h"
#include "ossm/state/network.h"
#include "ossm/state/state.h"
#include "services/communication/mqtt.h"
#include "utils/tls_session.hpp"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif

#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "main"
#endif

// Fleet builds take their updates from the builder's own server.
#ifdef FLEET_SERVER
#define OSSM_UPDATE_SERVER FLEET_SERVER
#else
#define OSSM_UPDATE_SERVER RAD_SERVER
#endif

// A TLS handshake keeps a core busy for seconds. At the default priority it
// starves the idle task, which trips the task watchdog; in fleet builds the
// update tasks run at idle priority instead.
#ifdef FLEET_SERVER
#define OSSM_UPDATE_TASK_PRIORITY tskIDLE_PRIORITY
#else
#define OSSM_UPDATE_TASK_PRIORITY 1
#endif

namespace {

std::uint32_t physicalFlashSizeBytes() {
    std::uint32_t size = 0;
    if (esp_flash_get_physical_size(nullptr, &size) == ESP_OK && size > 0) {
        return size;
    }
    return ESP.getFlashChipSize();
}

std::uint32_t otaSlotSizeBytes() {
    const esp_partition_t *partition =
        esp_ota_get_next_update_partition(nullptr);
    return partition == nullptr ? 0 : partition->size;
}

const char *currentPartitionLayout() {
    const esp_partition_t *app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
    const esp_partition_t *app1 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
    if (app0 == nullptr || app1 == nullptr) return "unknown";
    if (app0->address == 0x10000 && app0->size == 0x780000 &&
        app1->address == 0x790000 && app1->size == 0x780000) {
        return "ossm-ota-16mb-v1";
    }
    if (app0->address == 0x10000 && app0->size == 0x1F0000 &&
        app1->address == 0x200000 && app1->size == 0x1F0000) {
        return "ossm-ota-v1";
    }
    return "unknown";
}

std::string runningFirmwareHash() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    unsigned char digest[32] = {};
    if (running == nullptr || esp_partition_get_sha256(running, digest) != ESP_OK) {
        return "";
    }
    return firmware::sha256Hex(digest);
}

firmware::DeviceReport makeDeviceReport() {
    firmware::DeviceReport report;
    report.deviceType = "ossm";
    report.deviceId = std::string(WiFi.macAddress().c_str());
    report.reportedTrack = FIRMWARE_TRACK;
    report.currentVersion = VERSION;
    report.currentBuild = FIRMWARE_BUILD_SHA;
    report.firmwareHash = runningFirmwareHash();
    report.chip = std::string(ESP.getChipModel());
    report.chipRevision = ESP.getChipRevision();
    report.chipCores = ESP.getChipCores();
    report.hardwareRevision = "ossm-v1";
    report.flashSizeBytes = physicalFlashSizeBytes();
    report.psramSizeBytes = ESP.getPsramSize();
    report.otaSlotSizeBytes = otaSlotSizeBytes();
    report.partitionLayout = currentPartitionLayout();
    firmware::provenance::reconcile(report);
    return report;
}

void failUpdate(const char *code, const String &detail) {
    ESP_LOGE(UPDATE_TAG, "Firmware update failed (%s): %s", code, detail.c_str());
    networkStatus.error = code;
    networkStatus.errorDetail = detail;
    stateMachine->process_event(UpdateFailed{});
}

// The complete HTTPS check and install runs isolated from the button task.
// TlsSession pauses MQTT for the duration (its TLS session would otherwise
// compete with the update client for heap) and refuses to start when the
// largest free block is below the measured TLS budget. Every exit is a state
// machine event so the display and the BLE state characteristic always show
// what happened; motor control is never invoked by this task.
// Decision the last check offered; consumed by the install task.
firmware::Decision offeredDecision;
bool offeredDecisionValid = false;

// HTTPS check only. TlsSession pauses MQTT for the duration (its TLS session
// would otherwise compete with the update client for heap) and refuses to
// start when the largest free block is below the measured TLS budget. Every
// exit is a state machine event so the display and the BLE state
// characteristic always show what happened; motor control is never invoked.
void runCheck() {
    networkStatus.clearError();
    networkStatus.clearUpdate();
    offeredDecisionValid = false;
    ESP_LOGW(UPDATE_TAG, "Update check started: %s %s (%s)", VERSION,
             FIRMWARE_BUILD_SHA, FIRMWARE_TRACK);

    if (WiFi.status() != WL_CONNECTED) {
        failUpdate("wifi", "Wi-Fi is disconnected");
        return;
    }

    TlsSession tls("update-check");
    if (!tls.ok()) {
        failUpdate("low-memory", tls.error());
        return;
    }

    firmware::Decision decision;
    String error;
    const firmware::DeviceReport report = makeDeviceReport();
    if (!firmware::postCheck(OSSM_UPDATE_SERVER, report, decision, error)) {
        failUpdate("check-failed", error);
        return;
    }
    firmware::provenance::observeCurrent(report, decision);
    firmware::provenance::stageUpdate(report, decision);

    ESP_LOGW(UPDATE_TAG,
             "Resolver assigned track=%s shouldUpdate=%s target=%s next=%s reason=%s",
             decision.assignedTrack.c_str(),
             decision.shouldUpdate ? "true" : "false",
             decision.targetVersion.c_str(), decision.nextHopVersion.c_str(),
             decision.reason.c_str());
    if (!decision.shouldUpdate) {
        stateMachine->process_event(UpdateUnavailable{});
        return;
    }

    offeredDecision = decision;
    offeredDecisionValid = true;
    networkStatus.targetVersion = decision.nextHopVersion.empty()
                                      ? decision.targetVersion.c_str()
                                      : decision.nextHopVersion.c_str();
    stateMachine->process_event(UpdateAvailable{});
}

// Download + install of the offered decision, after confirmation. Same TLS
// rules as the check. Reboots on success.
void runInstall() {
    networkStatus.clearError();
    if (!offeredDecisionValid) {
        failUpdate("install-failed", "no update was offered");
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        failUpdate("wifi", "Wi-Fi is disconnected");
        return;
    }

    TlsSession tls("update-install");
    if (!tls.ok()) {
        failUpdate("low-memory", tls.error());
        return;
    }

    // Everything an update does is logged at warning level so it reaches the
    // dashboard and is kept with the update.
    for (std::size_t index = 0; index < offeredDecision.artifactCount; ++index) {
        const auto &artifact = offeredDecision.artifacts[index];
        ESP_LOGW(UPDATE_TAG, "Installing %s: %u bytes", artifact.role.c_str(),
                 static_cast<unsigned>(artifact.sizeBytes));
    }
    ESP_LOGW(UPDATE_TAG, "Wi-Fi %d dBm, internal free=%u largest=%u",
             WiFi.RSSI(),
             static_cast<unsigned>(
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    firmware::installProgressHook = [](std::uint32_t done, std::uint32_t total) {
        static std::uint32_t lastTenth = 99;
        const std::uint32_t tenth = total == 0 ? 0 : (done * 10ULL) / total;
        if (done == 0 || tenth != lastTenth) {
            lastTenth = tenth;
            ESP_LOGW(UPDATE_TAG,
                     "Download %u%%: %u of %u bytes, free=%u lowest=%u, %d dBm",
                     static_cast<unsigned>(tenth * 10),
                     static_cast<unsigned>(done), static_cast<unsigned>(total),
                     static_cast<unsigned>(heap_caps_get_free_size(
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                     static_cast<unsigned>(heap_caps_get_minimum_free_size(
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                     WiFi.RSSI());
        }
    };
    const int64_t installStarted = esp_timer_get_time();

    String error;
    if (!firmware::installApplicationAndFilesystem(offeredDecision, error)) {
        ESP_LOGE(UPDATE_TAG,
                 "Install stopped after %lld ms: %s (free=%u lowest=%u, Wi-Fi %s)",
                 static_cast<long long>((esp_timer_get_time() - installStarted) /
                                        1000),
                 error.c_str(),
                 static_cast<unsigned>(
                     heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_minimum_free_size(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                 WiFi.status() == WL_CONNECTED ? "connected" : "DISCONNECTED");
        failUpdate("install-failed", error);
        return;
    }
    ESP_LOGW(UPDATE_TAG, "Install finished in %lld ms",
             static_cast<long long>((esp_timer_get_time() - installStarted) /
                                    1000));

    ESP_LOGW(UPDATE_TAG, "Verified firmware installed; restarting");
    esp_restart();
}

void updateTask(void *pvParameters) {
    runCheck();  // TlsSession restores MQTT when this returns
    vTaskDelete(nullptr);
}

void installTask(void *pvParameters) {
    runInstall();
    vTaskDelete(nullptr);
}

}  // namespace


void ossmConfirmRunningImage() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running == nullptr ||
        esp_ota_get_state_partition(running, &state) != ESP_OK ||
        state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }

    const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
    if (result == ESP_OK) {
        ESP_LOGW(UPDATE_TAG, "Confirmed pending OTA application");
    } else {
        ESP_LOGE(UPDATE_TAG, "Failed to confirm pending OTA application: %s",
                 esp_err_to_name(result));
    }
}

// With BLE connected and MQTT up the largest free block is ~9 KB (measured
// 2026-09-09), smaller than these tasks' stacks. Pausing MQTT first frees
// its TLS buffers; the TlsSession inside the task adopts the pause.
void ossmStartUpdate() {
    pauseMqttForTls();
    if (xTaskCreatePinnedToCore(updateTask, "updateTask",
                                20 * configMINIMAL_STACK_SIZE, nullptr, OSSM_UPDATE_TASK_PRIORITY,
                                nullptr, 0) != pdPASS) {
        resumeMqttAfterTls();
        failUpdate("low-memory", "could not create update task");
    }
}

void ossmStartInstall() {
    pauseMqttForTls();
    if (xTaskCreatePinnedToCore(installTask, "installTask",
                                20 * configMINIMAL_STACK_SIZE, nullptr, OSSM_UPDATE_TASK_PRIORITY,
                                nullptr, 0) != pdPASS) {
        resumeMqttAfterTls();
        failUpdate("low-memory", "could not create install task");
    }
}
