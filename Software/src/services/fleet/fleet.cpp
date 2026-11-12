#include "fleet.h"

#if defined(FLEET_SERVER) && defined(FLEET_KEY)

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <vector>

#include "constants/Menu.h"
#include "constants/Version.h"
#include "ossm/Events.h"
#include "ossm/state/menu.h"
#include "ossm/state/network.h"
#include "ossm/state/state.h"
#include "utils/https_client.hpp"
#include "utils/tls_session.hpp"
#include "utils/update.h"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif
#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "main"
#endif

namespace sml = boost::sml;
using namespace sml;

namespace {

const char *const TAG = "FLEET";

constexpr uint32_t FIRST_CHECKIN_MS = 45 * 1000;
constexpr uint32_t BUSY_RETRY_MS = 30 * 1000;
constexpr uint32_t MIN_INTERVAL_S = 60;
constexpr uint32_t MAX_INTERVAL_S = 3600;
constexpr uint32_t DEFAULT_INTERVAL_S = 300;
constexpr uint32_t CHECK_TIMEOUT_MS = 120 * 1000;
constexpr uint32_t INSTALL_TIMEOUT_MS = 10 * 60 * 1000;

struct Command {
    int id;
    String type;
};
struct Result {
    int id;
    bool ok;
    String detail;
};

// The scheduler task waits for each worker before touching these, so the two
// tasks never use them at the same time.
std::vector<Command> received;
std::vector<Result> results;
uint32_t intervalSeconds = DEFAULT_INTERVAL_S;
volatile bool workerDone = false;

bool inMenu() { return stateMachine->is("menu.idle"_s); }
bool inError() {
    return stateMachine->is("error.idle"_s) || stateMachine->is("error.help"_s);
}
// The only states in which the motor is guaranteed not to be in use.
bool isIdle() { return inMenu() || inError(); }

const char *stateName() { return inMenu() ? "menu.idle" : "error"; }

void checkin() {
    JsonDocument request;
    request["deviceId"] = WiFi.macAddress();
    request["version"] = VERSION;
    request["buildSha"] = FIRMWARE_BUILD_SHA;
    request["track"] = FIRMWARE_TRACK;
    request["chip"] = ESP.getChipModel();
    request["state"] = stateName();
    request["uptimeSeconds"] =
        static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
    JsonArray reported = request["results"].to<JsonArray>();
    for (const Result &result : results) {
        JsonObject item = reported.add<JsonObject>();
        item["id"] = result.id;
        item["ok"] = result.ok;
        item["detail"] = result.detail;
    }
    String body;
    serializeJson(request, body);

    const HttpsHeaders headers = {{"X-Fleet-Key", String(FLEET_KEY)}};
    int status = 0;
    String response;
    String error;
    if (!httpsPostJson(String(FLEET_SERVER) + "/api/fleet/v1/checkin", body,
                       status, response, error, 15000, &headers) ||
        status != 200) {
        ESP_LOGW(TAG, "Check-in failed: HTTP %d %s", status, error.c_str());
        return;
    }
    results.clear();

    JsonDocument reply;
    if (deserializeJson(reply, response)) {
        ESP_LOGW(TAG, "Check-in reply was not valid JSON");
        return;
    }
    intervalSeconds = constrain(reply["nextCheckinSeconds"] | DEFAULT_INTERVAL_S,
                                MIN_INTERVAL_S, MAX_INTERVAL_S);
    for (JsonObjectConst item : reply["commands"].as<JsonArrayConst>()) {
        if (item["id"].is<int>() && item["type"].is<const char *>()) {
            received.push_back({item["id"].as<int>(),
                                String(item["type"].as<const char *>())});
        }
    }
    ESP_LOGW(TAG, "Checked in: %u command(s), next in %u s",
             static_cast<unsigned>(received.size()),
             static_cast<unsigned>(intervalSeconds));
}

// The HTTPS call needs a large stack that the heap cannot spare permanently,
// so it runs in a short-lived task, like the firmware's own update check.
void workerTask(void *) {
    {
        TlsSession tls("fleet-checkin");  // restores MQTT when it ends
        if (tls.ok()) {
            checkin();
        } else {
            ESP_LOGW(TAG, "Check-in skipped: %s", tls.error().c_str());
        }
    }
    workerDone = true;
    vTaskDelete(nullptr);
}

void runWorker() {
    workerDone = false;
    pauseMqttForTls();  // free MQTT's TLS buffers before taking the stack
    if (xTaskCreatePinnedToCore(workerTask, "fleetWorker",
                                20 * configMINIMAL_STACK_SIZE, nullptr, 1,
                                nullptr, 0) != pdPASS) {
        resumeMqttAfterTls();
        ESP_LOGW(TAG, "Check-in skipped: could not create worker task");
        return;
    }
    for (int waited = 0; !workerDone && waited < 600; waited++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

template <typename Done>
bool waitFor(uint32_t timeoutMs, Done done) {
    for (uint32_t waited = 0; waited < timeoutMs; waited += 250) {
        if (done()) return true;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return done();
}

void fail(const Command &command, const String &detail) {
    ESP_LOGW(TAG, "Command %d (%s) failed: %s", command.id,
             command.type.c_str(), detail.c_str());
    results.push_back({command.id, false, detail});
}

// From the menu the update goes through the normal update screens, so the
// display shows what is happening and the menu cannot start a session
// meanwhile. The fleet task presses "install" itself instead of waiting for
// the owner.
void updateFromMenu(const Command &command) {
    menuState.currentOption = Menu::UpdateOSSM;
    stateMachine->process_event(ButtonPress{});

    const bool settled = waitFor(CHECK_TIMEOUT_MS, [] {
        return !stateMachine->is("update.checking"_s) &&
               !stateMachine->is("update"_s);
    });
    if (settled && stateMachine->is("update.available"_s)) {
        stateMachine->process_event(ButtonPress{});
        // A successful install restarts the machine, so returning from this
        // wait means it failed or timed out.
        waitFor(INSTALL_TIMEOUT_MS,
                [] { return !stateMachine->is("update.installing"_s); });
        fail(command, "install-failed: " + networkStatus.error);
    } else if (stateMachine->is("update.idle"_s)) {
        fail(command, "server offered no update for this machine");
    } else if (stateMachine->is("update.failed"_s)) {
        fail(command, "check-failed: " + networkStatus.error);
    } else {
        fail(command, "update did not start");
    }
    stateMachine->process_event(ReturnToMenu{});
}

// On the error screen (for example after a failed homing) the menu is not
// reachable, but this is exactly when a fix needs to get in. The motor is
// stopped in this state, so the update runs directly.
void updateFromError(const Command &command) {
    networkStatus.clearError();
    networkStatus.clearUpdate();
    ossmStartUpdate();
    const bool settled = waitFor(CHECK_TIMEOUT_MS, [] {
        return !networkStatus.targetVersion.isEmpty() ||
               !networkStatus.error.isEmpty();
    });
    if (!settled || networkStatus.targetVersion.isEmpty()) {
        fail(command, settled ? "check-failed: " + networkStatus.error
                              : String("server offered no update for this machine"));
        return;
    }
    ossmStartInstall();
    waitFor(INSTALL_TIMEOUT_MS, [] { return !networkStatus.error.isEmpty(); });
    fail(command, "install-failed: " + networkStatus.error);
}

void execute(const Command &command) {
    if (!isIdle()) {
        fail(command, "machine was in use; send the command again");
        return;
    }
    ESP_LOGW(TAG, "Running command %d: %s", command.id, command.type.c_str());
    if (command.type == "restart") {
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP.restart();
    } else if (command.type == "update") {
        if (inMenu()) {
            updateFromMenu(command);
        } else {
            updateFromError(command);
        }
    } else {
        fail(command, "unknown command");
    }
}

void fleetTask(void *) {
    uint32_t delayMs = FIRST_CHECKIN_MS;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
        if (WiFi.status() != WL_CONNECTED || !isIdle()) {
            delayMs = BUSY_RETRY_MS;
            continue;
        }
        runWorker();
        const std::vector<Command> commands = received;
        received.clear();
        for (const Command &command : commands) execute(command);
        delayMs = intervalSeconds * 1000;
    }
}

}  // namespace

void initFleet() {
    if (xTaskCreatePinnedToCore(fleetTask, "fleetTask", 4096, nullptr, 1,
                                nullptr, 0) != pdPASS) {
        ESP_LOGE(TAG, "Could not create fleet task");
    }
}

#else

void initFleet() {}

#endif
