#include "board.h"

#include <Preferences.h>

bool USE_LATENCY_COMPENSATION = false;
bool USE_SPEED_KNOB_AS_LIMIT = true;

// Opening an NVS namespace read-only fails with
// "[E][Preferences.cpp:50] begin(): nvs_open failed: NOT_FOUND" until
// something has been written to it. On a fresh (or erased) device that error
// is logged on every boot and on every device-name / WiFi-status read.
// Opening each namespace read-write once creates it, so later read-only
// opens succeed and fall back to their defaults quietly.
static void ensureNvsNamespaces() {
    static const char *const namespaces[] = {"UserConfig", "wifi", "fw-prov",
                                             "radble"};
    for (const char *name : namespaces) {
        Preferences preferences;
        if (preferences.begin(name, false)) {
            preferences.end();
        } else {
            ESP_LOGW("BOARD", "Could not create NVS namespace '%s'", name);
        }
    }
}

void initBoard() {
    Serial.begin(115200);
    ensureNvsNamespaces();

    pinMode(Pins::Remote::encoderSwitch,
            INPUT_PULLDOWN);  // Rotary Encoder Pushbutton

    pinMode(Pins::GPIO::pin1, OUTPUT);
    pinMode(Pins::GPIO::pin2, OUTPUT);
    pinMode(Pins::GPIO::pin3, OUTPUT);
    pinMode(Pins::GPIO::pin4, OUTPUT);

    pinMode(Pins::Driver::motorEnablePin, OUTPUT);
    pinMode(Pins::Wifi::resetPin, INPUT_PULLDOWN);
    pinMode(Pins::Remote::speedPotPin, INPUT);
    adcAttachPin(Pins::Remote::speedPotPin);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // allows us to read almost full 3.3V range
    initStepper();
    initEncoder();
    initLED();
}
