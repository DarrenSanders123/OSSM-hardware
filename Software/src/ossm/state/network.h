#ifndef OSSM_STATE_NETWORK_H
#define OSSM_STATE_NETWORK_H

#include <Arduino.h>

/**
 * Outcome of the last network job (update check, pairing) so it can be
 * shown on the display and reported over the BLE state characteristic.
 *
 * error codes: "wifi" | "low-memory" | "check-failed" | "install-failed" |
 *              "pairing-failed"
 */
struct NetworkStatus {
    String error;
    String pairingCode;
    bool isPaired = false;
    // Version the resolver offered; set while waiting in update.available.
    String targetVersion;

    // What went wrong, in words, for the log and the dashboard ("artifact
    // returned HTTP 404"). `error` stays the short code shown on screen.
    String errorDetail;

    void clearError() {
        error = "";
        errorDetail = "";
    }
    // Set when an update check finished and the server offered nothing.
    bool noUpdate = false;

    void clearUpdate() {
        targetVersion = "";
        noUpdate = false;
    }
    void clearPairing() {
        pairingCode = "";
        isPaired = false;
    }
};

extern NetworkStatus networkStatus;

#endif  // OSSM_STATE_NETWORK_H
