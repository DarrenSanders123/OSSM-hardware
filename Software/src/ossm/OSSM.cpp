#include "OSSM.h"

#include "FirmwareProvenance.h"

#include "command/commands.hpp"
#include "ossm/state/ble.h"
#include "ossm/state/calibration.h"
#include "ossm/state/menu.h"
#include "ossm/state/network.h"
#include "ossm/state/session.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "ossm/state_json.hpp"
#include "services/communication/mqtt.h"
#include "services/communication/queue.h"
#include "services/encoder.h"
#include "services/fleet/fleet.h"
#include "services/stepper.h"

namespace sml = boost::sml;
using namespace sml;

// Global OSSM pointer (kept for backward compatibility during migration)
OSSM *ossm = nullptr;

// Static member definition - now forwards to global settings
SettingPercents OSSM::setting = {.speed = 0,
                                 .stroke = 50,
                                 .sensation = 50,
                                 .depth = 10,
                                 .buffer = 100,
                                 .pattern = StrokePatterns::SimpleStroke};

OSSM::OSSM() {
    // Initialize global state from OSSM::setting
    settings = OSSM::setting;
}

bool OSSM::triggerMenuActionFromBle(Menu option) {
    if (stateMachine == nullptr) return false;
    bleState.remoteMenuAction = true;
    // A second go:update while an update is offered is the install confirmation.
    if (option == Menu::UpdateOSSM && stateMachine->is("update.available"_s)) {
        stateMachine->process_event(ButtonPress{});
        return true;
    }
    // Leave whatever we are doing: play modes honour ReturnToMenu, the simple
    // info/result pages (update.idle, pairing.failed, help...) exit on a
    // button press.
    if (!stateMachine->is("menu.idle"_s)) {
        stateMachine->process_event(ReturnToMenu{});
    }
    if (!stateMachine->is("menu.idle"_s)) {
        stateMachine->process_event(ButtonPress{});
    }
    if (!stateMachine->is("menu.idle"_s)) {
        ESP_LOGW("OSSM", "BLE menu action %d refused: not in a menu", (int)option);
        bleState.remoteMenuAction = false;
        return false;
    }
    menuState.currentOption = option;
    stateMachine->process_event(ButtonPress{});
    return true;
}

void OSSM::ble_click(String commandString) {
    CommandValue command = commandFromString(commandString);
    ESP_LOGD("OSSM", "COMMAND: %d", command.command);

    String currentState;
    if (stateMachine != nullptr) {
        stateMachine->visit_current_states(
            [&currentState](auto state) { currentState = state.c_str(); });
    }

    // A machine that could not home sits on its error screen, where any
    // button press advances the help page and the second one restarts it.
    // A remote starts a mode (twice, on purpose) as soon as it connects, so
    // without this guard connecting a remote restarts a machine that cannot
    // home, for example a bare board with no motor. Commands that do not
    // start a mode are untouched, and the machine can still be restarted
    // with its own button (two presses) or from the fleet dashboard.
    if (currentState.startsWith("error")) {
        // The menu's update flow cannot run from here, so a remote's request
        // is run in the background instead (fleet builds; otherwise ignored).
        if (command.command == Commands::goToUpdate) {
            fleetStartUpdateFromRemote();
            return;
        }
        // Pairing runs from the menu, so from here it could only press through
        // the help page, and the second press restarts.
        if (command.command == Commands::goToStrokeEngine ||
            command.command == Commands::goToSimplePenetration ||
            command.command == Commands::goToStreaming ||
            command.command == Commands::goToPairing) {
            ESP_LOGW("OSSM", "Ignored %s: machine is in %s",
                     commandString.c_str(), currentState.c_str());
            return;
        }
        // A restart was asked for: do it, instead of needing two button
        // presses that the remote never sends.
        if (command.command == Commands::goToRestart) {
            ESP_LOGW("OSSM", "Restarting on request from %s",
                     currentState.c_str());
            vTaskDelay(pdMS_TO_TICKS(300));
            ESP.restart();
        }
    }

    switch (command.command) {
        case Commands::goToStrokeEngine:
            menuState.currentOption = Menu::StrokeEngine;
            if (stateMachine != nullptr) {
                stateMachine->process_event(ButtonPress{});
            }
            break;
        case Commands::goToSimplePenetration:
            menuState.currentOption = Menu::SimplePenetration;
            if (stateMachine != nullptr) {
                stateMachine->process_event(ButtonPress{});
            }
            break;
        case Commands::goToStreaming:
            menuState.currentOption = Menu::Streaming;
            if (stateMachine != nullptr) {
                stateMachine->process_event(ButtonPress{});
            }
            break;
        case Commands::goToMenu:
            if (stateMachine != nullptr) {
                stateMachine->process_event(ReturnToMenu{});
            }
            break;
        case Commands::goToRestart:
            triggerMenuActionFromBle(Menu::Restart);
            break;
        case Commands::goToUpdate:
            triggerMenuActionFromBle(Menu::UpdateOSSM);
            break;
        case Commands::goToPairing:
            triggerMenuActionFromBle(Menu::Pairing);
            break;
        case Commands::setSpeed:
            // BLE devices can be trusted to send true value
            // and can bypass potentiomer smoothing logic
            bleState.lastSpeedCommandWasFromBLE = true;
            // Use speed knob config to determine how to handle BLE speed
            // command
            settings.speedBLE = command.value;
            break;
        case Commands::setStroke:
            session.playControl = PlayControls::STROKE;
            encoder.setEncoderValue(command.value);
            settings.stroke = command.value;
            break;
        case Commands::setDepth:
            session.playControl = PlayControls::DEPTH;
            encoder.setEncoderValue(command.value);
            settings.depth = command.value;
            break;
        case Commands::setSensation:
            session.playControl = PlayControls::SENSATION;
            encoder.setEncoderValue(command.value);
            settings.sensation = command.value;
            break;
        case Commands::setBuffer:
            session.playControl = PlayControls::BUFFER;
            encoder.setEncoderValue(command.value);
            settings.buffer = command.value;
            break;
        case Commands::setPattern:
            settings.pattern = static_cast<StrokePatterns>(command.value % 7);
            break;
        case Commands::streamPosition:
            // Position (0-100)
            targetQueue.push({
                static_cast<uint8_t>(command.value),
                static_cast<uint16_t>(command.time),
                std::chrono::steady_clock::now()});
            break;
        case Commands::setWifi:
        case Commands::ignore:
            break;
    }
}

String OSSM::getStateFingerprint() {
    String currentState;
    if (stateMachine != nullptr) {
        stateMachine->visit_current_states(
            [&currentState](auto state) { currentState = state.c_str(); });
    }

    String output = currentState + ":";
    output += String((int)settings.speed) + ":";
    output += String((int)settings.stroke) + ":";
    output += String((int)settings.sensation) + ":";
    output += String((int)settings.depth) + ":";
    output += String(static_cast<int>(settings.pattern)) + ":";
    output += networkStatus.error + ":" + networkStatus.pairingCode + ":" +
              String(networkStatus.isPaired ? 1 : 0) + ":" + networkStatus.targetVersion + ":";
    if (const char *virtualState = fleetVirtualState()) output += virtualState;
    output += sessionId;
    return output;
}

// ┌──────────────────────────────────────────────────────────────────────┐
// │ MQTT TELEMETRY PAYLOAD — SHARED CONTRACT WITH RAD DASHBOARD        │
// │                                                                    │
// │ This payload is published via MQTT to `ossm/{macAddress}` and      │
// │ received by the Dashboard at:                                      │
// │   rad-app/src/app/api/lockbox/event/ossm/[mac]/route.ts            │
// │                                                                    │
// │ The Dashboard validates it with a Zod schema (payloadSchema).      │
// │ ANY change here MUST be mirrored in that Zod schema, and vice      │
// │ versa, or telemetry ingestion will silently fail (400 Bad Request). │
// │                                                                    │
// │ Required fields (all must be present):                             │
// │   timestamp  : number   — millis() uptime                         │
// │   state      : string   — Boost.SML state name                    │
// │   speed      : integer  — cast from float                         │
// │   stroke     : integer  — cast from float                         │
// │   sensation  : integer  — cast from float                         │
// │   depth      : integer  — cast from float                         │
// │   pattern    : integer  — StrokePatterns enum ordinal             │
// │   position   : number   — stepper position in mm (float)          │
// │   sessionId  : UUID     — regenerated each time a play mode starts  │
// │                                                                    │
// │ Optional fields:                                                   │
// │   meta       : string   — JSON-encoded metadata (optional)         │
// │                                                                    │
// │ This is also sent over BLE via NimBLE notifications.               │
// │ See: test/test_mqtt_payload/ for contract tests.                   │
// └──────────────────────────────────────────────────────────────────────┘
String OSSM::getCurrentState() {
    StateJsonInput input;
    if (stateMachine != nullptr) {
        stateMachine->visit_current_states(
            [&input](auto state) { input.state = state.c_str(); });
    }

    // While an update asked for by a remote runs from the error screen, tell
    // the remote its update states (no-op in builds without the fleet client).
    if (const char *virtualState = fleetVirtualState()) input.state = virtualState;

    float positionMm = float(stepper->getCurrentPosition()) / float(1_mm);
    if (isnan(positionMm)) positionMm = 0.0f;

    const String provenanceId =
        firmware::provenance::currentTokenId().c_str();
    input.timestamp = (unsigned long)millis();
    input.speed = (int)settings.speed;
    input.stroke = (int)settings.stroke;
    input.sensation = (int)settings.sensation;
    input.depth = (int)settings.depth;
    input.buffer = (int)settings.buffer;
    input.pattern = static_cast<int>(settings.pattern);
    input.position = positionMm;
    input.sessionId = sessionId;
    input.error = networkStatus.error;
    input.pairingCode = networkStatus.pairingCode;
    input.isPaired = networkStatus.isPaired;
    input.targetVersion = networkStatus.targetVersion;
    input.firmwareProvenanceId = provenanceId;
    return buildStateJson(input);
}
