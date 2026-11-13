#include "fleet.h"

#if defined(FLEET_SERVER) && defined(FLEET_KEY) && defined(FLEET_LINK_URL)

#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp32/rom/ets_sys.h>
#include <esp32/rom/uart.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_websocket_client.h>

#include <vector>

#include "command/command_pattern.hpp"
#include "constants/Config.h"
#include "constants/Menu.h"
#include "constants/Pins.h"
#include "constants/Version.h"
#include "fleet_roots.h"
#include "ossm/Events.h"
#include "ossm/OSSM.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/menu.h"
#include "ossm/state/network.h"
#include "ossm/state/state.h"
#include <RadBle.h>

#include "services/communication/nimble.h"
#include "services/led.h"
#include "services/wm.h"
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

constexpr uint32_t STATUS_INTERVAL_MS = 30 * 1000;
constexpr uint32_t FALLBACK_AFTER_MS = 3 * 60 * 1000;
constexpr uint32_t FALLBACK_POLL_MS = 5 * 60 * 1000;
constexpr uint32_t CHECK_TIMEOUT_MS = 120 * 1000;
constexpr uint32_t INSTALL_TIMEOUT_MS = 10 * 60 * 1000;
constexpr uint32_t HEALTHY_AFTER_MS = 5 * 60 * 1000;
constexpr uint32_t CRASHES_FOR_SAFE_MODE = 3;

// ---------------------------------------------------------------------------
// Crash-loop guard. If the machine crashes several times in a row the live
// link stays off and only the simple HTTPS polling runs, so a bad build can
// still be replaced remotely. The counters live in RTC memory, which keeps
// its contents across crashes and restarts but not across a power cut.
// ---------------------------------------------------------------------------
constexpr uint32_t RTC_MAGIC = 0xF1EE7C0D;
RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint32_t crashStreak;

// An update needs a large free block for TLS. A machine that has been running
// a while has a fragmented heap and a download then runs out of memory, so an
// update first restarts the machine and runs right after boot.
constexpr uint32_t UPDATE_AFTER_RESTART = 0x0D1A7E01;
constexpr uint32_t HEAP_FRESH_MS = 90 * 1000;
RTC_NOINIT_ATTR uint32_t updateAfterRestartMagic;

// Log lines waiting to be sent. Also in RTC memory, so the lines written
// just before a crash are still there to send after the restart.
constexpr size_t LOG_RING_BYTES = 4096;
RTC_NOINIT_ATTR char logRing[LOG_RING_BYTES];
RTC_NOINIT_ATTR uint32_t logHead;  // next byte to write
RTC_NOINIT_ATTR uint32_t logLen;   // bytes in the ring
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
bool safeMode = false;

const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "power on";
        case ESP_RST_SW: return "restart";
        case ESP_RST_PANIC: return "crash";
        case ESP_RST_INT_WDT: return "watchdog (interrupt)";
        case ESP_RST_TASK_WDT: return "watchdog (task)";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_BROWNOUT: return "power dip (brownout)";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_EXT: return "reset pin";
        default: return "unknown";
    }
}

void evaluateLastReset() {
    resetReason = esp_reset_reason();
    const bool crashed =
        resetReason == ESP_RST_PANIC || resetReason == ESP_RST_INT_WDT ||
        resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT ||
        resetReason == ESP_RST_BROWNOUT;
    if (rtcMagic != RTC_MAGIC || logLen > LOG_RING_BYTES ||
        logHead >= LOG_RING_BYTES) {
        rtcMagic = RTC_MAGIC;
        crashStreak = 0;
        logHead = 0;
        logLen = 0;
    }
    crashStreak = crashed ? crashStreak + 1 : 0;
    safeMode = crashStreak >= CRASHES_FOR_SAFE_MODE;
}

// ---------------------------------------------------------------------------
// Log capture: everything the firmware prints at warning level or above is
// copied into logRing on its way to the serial port.
// ---------------------------------------------------------------------------
void IRAM_ATTR logPush(const char *data, size_t length) {
    portENTER_CRITICAL_SAFE(&logMux);
    for (size_t index = 0; index < length; index++) {
        const char character = data[index];
        if (character == '\r') continue;
        // When full the oldest text is overwritten: the end of an update, or
        // what happened just before a crash, is what is worth reading.
        logRing[logHead] = character;
        logHead = (logHead + 1) % LOG_RING_BYTES;
        if (logLen < LOG_RING_BYTES) logLen++;
    }
    portEXIT_CRITICAL_SAFE(&logMux);
}

// Arduino-style log lines (log_w, ESP_LOGW in application code) end up here.
void IRAM_ATTR capturePutc(char character) {
    uart_tx_one_char(character);
    logPush(&character, 1);
}

// ESP-IDF's own components log through here. They are chatty at info level,
// so only their warnings and errors are kept.
int captureVprintf(const char *format, va_list arguments) {
    char line[192];
    va_list copy;
    va_copy(copy, arguments);
    const int written = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    if (written > 0 && (line[0] == 'E' || line[0] == 'W')) {
        logPush(line, min(static_cast<size_t>(written), sizeof(line) - 1));
    }
    return vprintf(format, arguments);
}

// Takes whole lines out of the ring, oldest first, up to `capacity` bytes.
size_t logDrain(char *out, size_t capacity) {
    size_t take = 0;
    portENTER_CRITICAL(&logMux);
    const size_t start = (logHead + LOG_RING_BYTES - logLen) % LOG_RING_BYTES;
    take = min(static_cast<size_t>(logLen), capacity);
    // Cut back to the last complete line.
    size_t complete = take;
    while (complete > 0 && logRing[(start + complete - 1) % LOG_RING_BYTES] != '\n') {
        complete--;
    }
    if (complete == 0 && logLen >= capacity) complete = take;  // one very long line
    take = complete;
    for (size_t index = 0; index < take; index++) {
        out[index] = logRing[(start + index) % LOG_RING_BYTES];
    }
    logLen -= take;
    portEXIT_CRITICAL(&logMux);
    return take;
}

// ---------------------------------------------------------------------------
// Identity: every machine creates its own random key once and keeps it. The
// server remembers it the first time it sees the machine, so nobody else can
// later connect under this machine's address.
// ---------------------------------------------------------------------------
String machineKey;
volatile bool fleetReady = false;

void loadMachineKey() {
    Preferences preferences;
    if (!preferences.begin("fleet", false)) return;
    if (preferences.isKey("secret")) {
        machineKey = preferences.getString("secret", "");
    }
    if (machineKey.length() != 64) {
        uint8_t raw[32];
        esp_fill_random(raw, sizeof(raw));  // Wi-Fi is up: true random numbers
        char hex[65];
        for (size_t index = 0; index < sizeof(raw); index++) {
            snprintf(hex + index * 2, 3, "%02x", raw[index]);
        }
        machineKey = hex;
        preferences.putString("secret", machineKey);
    }
    preferences.end();
}

// ---------------------------------------------------------------------------
// Machine state helpers
// ---------------------------------------------------------------------------
String stateName() {
    String name;
    if (stateMachine != nullptr) {
        stateMachine->visit_current_states(
            [&name](auto state) { name = state.c_str(); });
    }
    return name;
}

int bleCentralCount() {
    NimBLEServer *bleServer = NimBLEDevice::getServer();
    return bleServer != nullptr ? bleServer->getConnectedCount() : 0;
}

bool inMenu() { return stateMachine->is("menu.idle"_s); }
bool inError() {
    return stateMachine->is("error.idle"_s) || stateMachine->is("error.help"_s);
}
// The states in which the motor is certainly not running a session.
bool isIdle() { return inMenu() || inError(); }
bool inSession() {
    const String name = stateName();
    return name.startsWith("simplePenetration") ||
           name.startsWith("strokeEngine") || name.startsWith("streaming");
}

String currentDeviceName();

void buildStatus(JsonDocument &status) {
    status["t"] = "status";
    status["version"] = VERSION;
    status["buildSha"] = FIRMWARE_BUILD_SHA;
    status["track"] = FIRMWARE_TRACK;
    status["chip"] = ESP.getChipModel();
    status["state"] = stateName();
    status["deviceName"] = currentDeviceName();
    status["uptimeSeconds"] =
        static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
    status["resetReason"] = resetReasonName(resetReason);
    status["crashes"] = crashStreak;
    status["safeMode"] = safeMode;

    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    status["heapFree"] = heap_caps_get_free_size(caps);
    status["heapLargest"] = heap_caps_get_largest_free_block(caps);
    status["heapMin"] = heap_caps_get_minimum_free_size(caps);

    if (WiFi.status() == WL_CONNECTED) {
        status["wifiSsid"] = WiFi.SSID();
        status["wifiRssi"] = WiFi.RSSI();
        status["wifiIp"] = WiFi.localIP().toString();
    }

    status["homed"] = calibration.isHomed;
    status["strokeMm"] =
        calibration.measuredStrokeSteps / Config::Driver::stepsPerMM;
    status["currentOffset"] = calibration.currentSensorOffset;
    status["currentNow"] =
        100.0f * analogRead(Pins::Driver::currentSensorPin) / 4096.0f -
        calibration.currentSensorOffset;
    if (inError()) status["error"] = errorState.message;

    status["bleCentrals"] = bleCentralCount();

    // Speed, stroke, depth, sensation, pattern as the remote sees them.
    if (ossm != nullptr) {
        JsonDocument live;
        if (!deserializeJson(live, ossm->getCurrentState())) {
            status["ossm"] = live;
        }
    }
}

// ---------------------------------------------------------------------------
// Live link: one WebSocket to the fleet server, kept open.
// ---------------------------------------------------------------------------
esp_websocket_client_handle_t linkClient = nullptr;
SemaphoreHandle_t linkMutex = nullptr;
QueueHandle_t linkInbox = nullptr;  // char* messages from the server
String linkHeaders;
volatile bool linkUp = false;
volatile bool linkPaused = false;
volatile bool statusRequested = false;
bool linkWanted = false;
uint32_t linkReconnects = 0;

void onLinkEvent(void *, esp_event_base_t, int32_t eventId, void *eventData) {
    const auto *event = static_cast<esp_websocket_event_data_t *>(eventData);
    switch (eventId) {
        case WEBSOCKET_EVENT_CONNECTED:
            linkUp = true;
            linkReconnects++;
            statusRequested = true;
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
        case WEBSOCKET_EVENT_CLOSED:
            linkUp = false;
            break;
        case WEBSOCKET_EVENT_DATA:
            // Commands are short text frames that arrive in one piece.
            if (event->op_code == 0x1 && event->payload_offset == 0 &&
                event->data_len == event->payload_len && event->data_len > 0 &&
                event->data_len < 1024) {
                char *copy = strndup(event->data_ptr, event->data_len);
                if (copy != nullptr &&
                    xQueueSend(linkInbox, &copy, 0) != pdTRUE) {
                    free(copy);
                }
            }
            break;
        default:
            break;
    }
}

void linkStart() {
    xSemaphoreTake(linkMutex, portMAX_DELAY);
    linkWanted = true;
    if (linkClient == nullptr) {
        linkHeaders = "X-Fleet-Device: " + WiFi.macAddress() +
                      "\r\nX-Fleet-Key: " FLEET_KEY "\r\nX-Fleet-Secret: " +
                      machineKey + "\r\nX-Fleet-Version: " VERSION "\r\n";
        esp_websocket_client_config_t config = {};
        config.uri = FLEET_LINK_URL;
        config.headers = linkHeaders.c_str();
        config.cert_pem = FLEET_LINK_ROOTS;
        config.buffer_size = 1536;
        config.task_stack = 6144;
        config.task_prio = 3;
        config.ping_interval_sec = 20;
        config.pingpong_timeout_sec = 60;
        linkClient = esp_websocket_client_init(&config);
        if (linkClient != nullptr) {
            esp_websocket_register_events(linkClient, WEBSOCKET_EVENT_ANY,
                                          onLinkEvent, nullptr);
        }
    }
    if (linkClient != nullptr && !linkPaused) {
        esp_websocket_client_start(linkClient);
    }
    xSemaphoreGive(linkMutex);
}

bool linkSend(const String &text) {
    if (!linkUp || linkClient == nullptr) return false;
    return esp_websocket_client_send_text(linkClient, text.c_str(),
                                          text.length(),
                                          pdMS_TO_TICKS(3000)) >= 0;
}

bool linkSendJson(const JsonDocument &document) {
    String text;
    serializeJson(document, text);
    return linkSend(text);
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
struct Command {
    int id = 0;
    String type;
    String text;
    bool force = false;
};
struct Result {
    int id;
    bool ok;
    String detail;
};

// Only the fleet task touches these.
std::vector<Result> results;
std::vector<Command> polled;
uint32_t pollIntervalMs = FALLBACK_POLL_MS;
volatile bool workerDone = false;

// The name the machine shows over Bluetooth and uses on the Wi-Fi network.
// Same rules and storage as the Bluetooth rename characteristic, so both ways
// of renaming agree.
String currentDeviceName() { return radble::loadDeviceName("OSSM"); }

bool validDeviceName(const String &name) {
    if (name.isEmpty() || name.length() > radble::DEVICE_NAME_MAX_BYTES) return false;
    if (name[0] == ' ' || name[0] == '-' || name[0] == '.' || name[0] == '_') return false;
    for (size_t index = 0; index < name.length(); index++) {
        const char c = name[index];
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == ' ' || c == '-' ||
                        c == '_' || c == '.';
        if (!ok) return false;
    }
    return true;
}

// A Wi-Fi host name allows letters, digits and dashes only.
String hostnameFor(const String &name) {
    String host;
    for (size_t index = 0; index < name.length() && host.length() < 32; index++) {
        const char c = name[index];
        const bool alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9');
        host += alnum ? c : '-';
    }
    return host.isEmpty() ? String("ossm") : host;
}

bool saveDeviceName(String name) {
    name.trim();
    if (!validDeviceName(name) || !radble::persistDeviceName(name)) return false;
    Preferences userConfig;
    if (userConfig.begin("UserConfig", false)) {
        userConfig.putString("DeviceName", name);
        userConfig.end();
    }
    return true;
}

// "code: what went wrong", for the dashboard.
String updateFailure() {
    return networkStatus.errorDetail.isEmpty()
               ? networkStatus.error
               : networkStatus.error + ": " + networkStatus.errorDetail;
}

void report(const Command &command, bool ok, const String &detail = "") {
    if (command.id == 0) return;
    if (!ok) {
        ESP_LOGW(TAG, "Command %d (%s) failed: %s", command.id,
                 command.type.c_str(), detail.c_str());
    }
    results.push_back({command.id, ok, detail});
}

void flushResults() {
    while (!results.empty()) {
        JsonDocument message;
        message["t"] = "result";
        message["id"] = results.front().id;
        message["ok"] = results.front().ok;
        if (!results.front().detail.isEmpty()) {
            message["detail"] = results.front().detail;
        }
        if (!linkSendJson(message)) return;  // keep it for the next attempt
        results.erase(results.begin());
    }
}

void flushLogs() {
    char chunk[900];
    const size_t length = logDrain(chunk, sizeof(chunk));
    if (length == 0) return;
    JsonDocument message;
    message["t"] = "log";
    JsonArray lines = message["lines"].to<JsonArray>();
    size_t start = 0;
    for (size_t index = 0; index <= length; index++) {
        if (index == length || chunk[index] == '\n') {
            if (index > start) lines.add(String(chunk + start, index - start));
            start = index + 1;
        }
    }
    if (lines.size() > 0) linkSendJson(message);
}

template <typename Done>
bool waitFor(uint32_t timeoutMs, Done done) {
    for (uint32_t waited = 0; waited < timeoutMs; waited += 250) {
        if (done()) return true;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return done();
}

// From the menu the update goes through the normal update screens, so the
// display shows what is happening and the menu cannot start a session
// meanwhile. The fleet task confirms the install itself instead of waiting
// for the owner to press the knob.
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
        report(command, false, updateFailure());
    } else if (stateMachine->is("update.idle"_s)) {
        report(command, false, "server offered no update for this machine");
    } else if (stateMachine->is("update.failed"_s)) {
        report(command, false, updateFailure());
    } else {
        report(command, false, "update did not start");
    }
    stateMachine->process_event(ReturnToMenu{});
}

// On the error screen (for example after a failed homing) the menu is not
// reachable, but this is exactly when a fix needs to get in. The motor is
// stopped in this state, so the update runs directly.
void updateFromError(const Command &command) {
    networkStatus.clearError();
    networkStatus.clearUpdate();
    if (millis() > HEAP_FRESH_MS) {
        vTaskDelay(pdMS_TO_TICKS(2500));  // the remote reads "update.checking"
        setUpdateActive(true);
        ESP_LOGW(TAG, "Restarting first: the update needs unfragmented memory");
        updateAfterRestartMagic = UPDATE_AFTER_RESTART;
        vTaskDelay(pdMS_TO_TICKS(300));
        ESP.restart();
    }
    // Let the remote read "update.checking", then free the Bluetooth
    // connection's memory: the TLS handshake does not fit beside it.
    vTaskDelay(pdMS_TO_TICKS(2500));
    bleReleaseForTls();
    ossmStartUpdate();
    const bool settled = waitFor(CHECK_TIMEOUT_MS, [] {
        return !networkStatus.targetVersion.isEmpty() ||
               !networkStatus.error.isEmpty() || networkStatus.noUpdate;
    });
    if (!settled || networkStatus.targetVersion.isEmpty()) {
        report(command, false,
               settled && !networkStatus.noUpdate
                   ? updateFailure()
                   : String("server offered no update for this machine"));
        return;
    }
    ossmStartInstall();
    waitFor(INSTALL_TIMEOUT_MS, [] { return !networkStatus.error.isEmpty(); });
    report(command, false, updateFailure());
}

// ---------------------------------------------------------------------------
// An update asked for by a remote while the machine is on its error screen.
// The remote follows the machine's state, so the phases are reported as the
// update states its own screens already know.
// ---------------------------------------------------------------------------
enum RemoteUpdatePhase { PHASE_NONE, PHASE_CHECKING, PHASE_INSTALLING, PHASE_UP_TO_DATE, PHASE_FAILED };
volatile RemoteUpdatePhase remoteUpdatePhase = PHASE_NONE;
constexpr uint32_t REMOTE_RESULT_HOLD_MS = 20 * 1000;  // long enough for the remote to read it

void remoteUpdateTask(void *) {
    remoteUpdatePhase = PHASE_CHECKING;
    networkStatus.clearError();
    networkStatus.clearUpdate();
    // Let the remote read "update.checking", then free the Bluetooth
    // connection's memory: the TLS handshake does not fit beside it (seen as
    // mbedtls -0x7F00). The remote reconnects when advertising returns.
    vTaskDelay(pdMS_TO_TICKS(2500));
    bleReleaseForTls();
    ossmStartUpdate();
    waitFor(CHECK_TIMEOUT_MS, [] {
        return !networkStatus.targetVersion.isEmpty() ||
               !networkStatus.error.isEmpty() || networkStatus.noUpdate;
    });
    if (networkStatus.targetVersion.isEmpty()) {
        remoteUpdatePhase = networkStatus.noUpdate ? PHASE_UP_TO_DATE : PHASE_FAILED;
    } else {
        remoteUpdatePhase = PHASE_INSTALLING;  // no second confirmation: it was asked for
        ossmStartInstall();
        waitFor(INSTALL_TIMEOUT_MS, [] { return !networkStatus.error.isEmpty(); });
        remoteUpdatePhase = PHASE_FAILED;  // a successful install restarts instead
    }
    ESP_LOGW(TAG, "Update asked for by a remote ended: %s",
             remoteUpdatePhase == PHASE_UP_TO_DATE ? "up to date" : updateFailure().c_str());
    bleResumeAfterTls();
    vTaskDelay(pdMS_TO_TICKS(REMOTE_RESULT_HOLD_MS));
    remoteUpdatePhase = PHASE_NONE;
    vTaskDelete(nullptr);
}

void execute(const Command &command) {
    if (command.type == "status") {
        statusRequested = true;
        return;
    }
    ESP_LOGW(TAG, "Remote command %d: %s %s", command.id, command.type.c_str(),
             command.text.c_str());

    if (command.type == "stop") {
        // Always allowed: stopping is the safe direction.
        stateMachine->process_event(EmergencyStop{});
        report(command, true, "state: " + stateName());
    } else if (command.type == "rename") {
        if (inSession()) {
            report(command, false, "machine is in use; try again when idle");
        } else if (!saveDeviceName(command.text)) {
            report(command, false, "that name is not allowed");
        } else {
            report(command, true, "renamed to " + command.text + "; restarting");
            flushResults();
            vTaskDelay(pdMS_TO_TICKS(800));
            ESP.restart();
        }
    } else if (command.type == "restart") {
        if (inSession() && !command.force) {
            report(command, false, "machine is in use; use force restart");
            return;
        }
        report(command, true);
        flushResults();
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP.restart();
    } else if (command.type == "update") {
        if (!isIdle()) {
            report(command, false, "machine is in use; try again when idle");
        } else if (millis() > HEAP_FRESH_MS) {
            report(command, true, "restarting first; the update runs right after");
            flushResults();
            // A connected remote reads this and shows "updating, do not restart".
            remoteUpdatePhase = PHASE_CHECKING;
            setUpdateActive(true);
            vTaskDelay(pdMS_TO_TICKS(3500));
            updateAfterRestartMagic = UPDATE_AFTER_RESTART;
            ESP.restart();
        } else if (inMenu()) {
            updateFromMenu(command);
        } else {
            updateFromError(command);
        }
    } else if (command.type == "home") {
        const bool handled = stateMachine->process_event(Home{});
        report(command, handled,
               handled ? "" : "cannot home from state " + stateName());
    } else if (command.type == "command") {
        // One line of the same control grammar the Bluetooth remote uses,
        // so the machine's own checks (speed at zero, homing) still apply.
        if (ossm == nullptr || !isValidBleCommand(command.text.c_str())) {
            report(command, false, "not a valid control command");
            return;
        }
        ossm->ble_click(command.text);
        vTaskDelay(pdMS_TO_TICKS(150));
        report(command, true, "state: " + stateName());
    } else {
        report(command, false, "unknown command");
    }
    statusRequested = true;
}

Command parseCommand(JsonObjectConst item) {
    Command command;
    command.id = item["id"] | 0;
    command.type = item["cmd"] | item["type"] | "";
    command.text = item["text"] | "";
    command.force = item["force"] | false;
    return command;
}

// ---------------------------------------------------------------------------
// HTTPS polling: the fallback when the live link cannot be used.
// ---------------------------------------------------------------------------
void poll() {
    JsonDocument request;
    buildStatus(request);
    request.remove("t");
    request.remove("ossm");
    request["deviceId"] = WiFi.macAddress();
    JsonArray reported = request["results"].to<JsonArray>();
    for (const Result &result : results) {
        JsonObject item = reported.add<JsonObject>();
        item["id"] = result.id;
        item["ok"] = result.ok;
        item["detail"] = result.detail;
    }
    String body;
    serializeJson(request, body);

    const HttpsHeaders headers = {{"X-Fleet-Key", String(FLEET_KEY)},
                                  {"X-Fleet-Secret", machineKey}};
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
    if (deserializeJson(reply, response)) return;
    pollIntervalMs =
        constrain(reply["nextCheckinSeconds"] | 300, 60, 3600) * 1000UL;
    for (JsonObjectConst item : reply["commands"].as<JsonArrayConst>()) {
        polled.push_back(parseCommand(item));
    }
}

// The HTTPS call needs a large stack that the heap cannot spare permanently,
// so it runs in a short-lived task, like the firmware's own update check.
void pollTask(void *) {
    {
        TlsSession tls("fleet-checkin");  // pauses and restores the live link
        if (tls.ok()) poll();
    }
    workerDone = true;
    vTaskDelete(nullptr);
}

void runPoll() {
    workerDone = false;
    pauseMqttForTls();
    // Priority 0 (the idle priority): the TLS handshake keeps a core busy for
    // seconds, and at a higher priority it starves the idle task, which then
    // trips the task watchdog.
    if (xTaskCreatePinnedToCore(pollTask, "fleetPoll",
                                20 * configMINIMAL_STACK_SIZE, nullptr,
                                tskIDLE_PRIORITY, nullptr, 0) != pdPASS) {
        resumeMqttAfterTls();
        return;
    }
    for (int waited = 0; !workerDone && waited < 600; waited++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    const std::vector<Command> commands = polled;
    polled.clear();
    for (const Command &command : commands) execute(command);
}

// ---------------------------------------------------------------------------
// Fleet task
// ---------------------------------------------------------------------------
void fleetTask(void *) {
    loadMachineKey();
    fleetReady = true;

    if (safeMode) {
        ESP_LOGE(TAG, "Safe mode after %u crashes in a row: live link off",
                 static_cast<unsigned>(crashStreak));
    } else {
        linkStart();
    }

    bool bootUpdatePending = updateAfterRestartMagic == UPDATE_AFTER_RESTART;
    updateAfterRestartMagic = 0;
    uint32_t lastStatus = 0;
    uint32_t lastLinkUp = millis();
    uint32_t lastPoll = 0;
    bool polledOnce = false;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(200));
        const uint32_t now = millis();

        if (bootUpdatePending) {
            if (now > 180 * 1000) {
                bootUpdatePending = false;
            } else if (WiFi.status() == WL_CONNECTED && isIdle()) {
                bootUpdatePending = false;
                ESP_LOGW(TAG, "Running the update that asked for a restart");
                if (inMenu()) {
                    updateFromMenu(Command{});
                } else {
                    fleetStartUpdateFromRemote();
                }
            }
        }

        // Running this long means the firmware is healthy.
        if (crashStreak != 0 && now > HEALTHY_AFTER_MS) crashStreak = 0;

        if (linkUp) {
            lastLinkUp = now;
            if (statusRequested || now - lastStatus >= STATUS_INTERVAL_MS) {
                statusRequested = false;
                lastStatus = now;
                JsonDocument status;
                buildStatus(status);
                status["linkReconnects"] = linkReconnects;
                linkSendJson(status);
            }
            flushResults();
            flushLogs();

            char *raw = nullptr;
            while (xQueueReceive(linkInbox, &raw, 0) == pdTRUE) {
                JsonDocument message;
                const bool valid = !deserializeJson(message, raw);
                free(raw);
                if (valid) execute(parseCommand(message.as<JsonObjectConst>()));
            }
            continue;
        }

        const bool fallback = safeMode || now - lastLinkUp > FALLBACK_AFTER_MS;
        const bool due = !polledOnce || now - lastPoll >= pollIntervalMs;
        // The HTTPS check needs a large block of memory that a connected remote
        // leaves too little of, so it only runs when no remote is connected.
        if (fallback && due && !linkPaused && WiFi.status() == WL_CONNECTED &&
            isIdle() && bleCentralCount() == 0) {
            polledOnce = true;
            lastPoll = now;
            runPoll();
        }
    }
}

// ---------------------------------------------------------------------------
// USB setup channel, used by the setup page in the browser. One JSON line in
// (prefixed FLEET?), one JSON line out (prefixed FLEET!).
// ---------------------------------------------------------------------------
void replySerial(const JsonDocument &reply) {
    // serializeJson() replaces the String it writes to, so the marker has to
    // be added around the result rather than before it.
    String json;
    serializeJson(reply, json);
    const String line = "\nFLEET!" + json + "\n";
    Serial.write(reinterpret_cast<const uint8_t *>(line.c_str()), line.length());
    Serial.flush();
}

void handleSerial(const String &json) {
    JsonDocument request;
    if (deserializeJson(request, json)) return;
    const String op = request["op"] | "";
    JsonDocument reply;
    reply["op"] = op;

    if (op == "info") {
        reply["fleet"] = 1;
        reply["mac"] = WiFi.macAddress();
        reply["version"] = VERSION;
        reply["buildSha"] = FIRMWARE_BUILD_SHA;
        reply["ready"] = static_cast<bool>(fleetReady);
        reply["deviceName"] = currentDeviceName();
        if (fleetReady) reply["secret"] = machineKey;
        JsonObject wifi = reply["wifi"].to<JsonObject>();
        wifi["connected"] = WiFi.status() == WL_CONNECTED;
        if (WiFi.status() == WL_CONNECTED) {
            wifi["ssid"] = WiFi.SSID();
            wifi["ip"] = WiFi.localIP().toString();
        }
    } else if (!fleetReady) {
        reply["ok"] = false;
        reply["error"] = "starting";
    } else if (op == "scan") {
        JsonArray networks = reply["networks"].to<JsonArray>();
        const int found = WiFi.scanNetworks();
        for (int index = 0; index < found && networks.size() < 15; index++) {
            const String ssid = WiFi.SSID(index);
            bool seen = ssid.isEmpty();
            for (JsonVariantConst known : networks) {
                if (ssid == known.as<const char *>()) seen = true;
            }
            if (!seen) networks.add(ssid);
        }
        WiFi.scanDelete();
        reply["ok"] = true;
    } else if (op == "name") {
        const String name = request["name"] | "";
        const bool ok = saveDeviceName(name);
        reply["ok"] = ok;
        replySerial(reply);
        if (ok) {
            vTaskDelay(pdMS_TO_TICKS(800));
            ESP.restart();
        }
        return;
    } else if (op == "wifi") {
        const String ssid = request["ssid"] | "";
        const String password = request["password"] | "";
        const bool ok = !ssid.isEmpty() &&
                        setWiFiCredentials(ssid, password) && connectWiFi();
        reply["ok"] = ok;
        if (ok) reply["ip"] = WiFi.localIP().toString();
    } else {
        reply["ok"] = false;
        reply["error"] = "unknown";
    }
    replySerial(reply);
}

void serialTask(void *) {
    String line;
    while (true) {
        while (Serial.available() > 0) {
            const char character = static_cast<char>(Serial.read());
            if (character == '\n' || character == '\r') {
                if (line.startsWith("FLEET?")) handleSerial(line.substring(6));
                line = "";
            } else if (line.length() < 400) {
                line += character;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

}  // namespace

void fleetEarlyInit() {
    evaluateLastReset();
    WiFi.setHostname(hostnameFor(currentDeviceName()).c_str());
    ets_install_putc1(capturePutc);
    esp_log_set_vprintf(captureVprintf);
    xTaskCreatePinnedToCore(serialTask, "fleetSerial", 6144, nullptr, 1,
                            nullptr, 0);
}

void initFleet() {
    linkMutex = xSemaphoreCreateMutex();
    linkInbox = xQueueCreate(8, sizeof(char *));
    if (linkMutex == nullptr || linkInbox == nullptr ||
        xTaskCreatePinnedToCore(fleetTask, "fleetTask", 8192, nullptr, 1,
                                nullptr, 0) != pdPASS) {
        ESP_LOGE(TAG, "Could not start the fleet task");
    }
}

bool fleetOwnsCloudLink() { return true; }

void fleetStartUpdateFromRemote() {
    if (remoteUpdatePhase != PHASE_NONE) return;  // already running
    ESP_LOGW(TAG, "A remote asked this machine to update");
    remoteUpdatePhase = PHASE_CHECKING;
    if (xTaskCreatePinnedToCore(remoteUpdateTask, "fleetRemoteUpd", 5120, nullptr,
                                1, nullptr, 0) != pdPASS) {
        remoteUpdatePhase = PHASE_NONE;
        ESP_LOGE(TAG, "Could not start the update asked for by a remote");
    }
}

const char *fleetVirtualState() {
    switch (remoteUpdatePhase) {
        case PHASE_CHECKING: return "update.checking";
        case PHASE_INSTALLING: return "update.installing";
        case PHASE_UP_TO_DATE: return "update.idle";
        case PHASE_FAILED: return "update.failed";
        default: return nullptr;
    }
}

// Called around every other job that needs the TLS memory (firmware update,
// pairing, the HTTPS fallback) and around homing. Safe to call repeatedly.
void fleetPauseLink() {
    if (linkMutex == nullptr) return;
    xSemaphoreTake(linkMutex, portMAX_DELAY);
    if (!linkPaused) {
        linkPaused = true;
        if (linkClient != nullptr && linkWanted) {
            esp_websocket_client_stop(linkClient);
            linkUp = false;
        }
    }
    xSemaphoreGive(linkMutex);
}

void fleetResumeLink() {
    if (linkMutex == nullptr) return;
    xSemaphoreTake(linkMutex, portMAX_DELAY);
    if (linkPaused) {
        linkPaused = false;
        if (linkClient != nullptr && linkWanted) {
            esp_websocket_client_start(linkClient);
        }
    }
    xSemaphoreGive(linkMutex);
}

#else

void fleetEarlyInit() {}
void initFleet() {}
bool fleetOwnsCloudLink() { return false; }
void fleetStartUpdateFromRemote() {}
const char *fleetVirtualState() { return nullptr; }
void fleetPauseLink() {}
void fleetResumeLink() {}

#endif
