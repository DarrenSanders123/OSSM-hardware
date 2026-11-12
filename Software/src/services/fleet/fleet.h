#ifndef OSSM_SERVICES_FLEET_H
#define OSSM_SERVICES_FLEET_H

/**
 * Fleet client: connects this machine to the builder's own server so it can
 * be updated, restarted, watched and operated remotely for support.
 *
 * Only active in builds that define FLEET_SERVER, FLEET_KEY and
 * FLEET_LINK_URL (the `fleet` PlatformIO environment, see fleet_build.py).
 * In every other build these functions do nothing.
 *
 * What a fleet build does differently:
 *  - Keeps one live connection to the fleet server instead of the Research
 *    and Desire cloud link (there is only memory for one). Bluetooth remotes
 *    are not affected.
 *  - Sends its status every 30 seconds and on request, and its warning and
 *    error log lines as they happen.
 *  - Runs commands from the server: restart, update, re-home, emergency stop
 *    and lines of the normal control grammar.
 *  - Takes firmware updates from the fleet server.
 *  - Answers the browser setup page over USB (firmware check, key, Wi-Fi).
 *  - After three crashes in a row it starts in safe mode: no live link, only
 *    a simple HTTPS check-in, so a bad build can still be replaced remotely.
 */

// Right after the board is initialised: log capture and the USB setup channel.
void fleetEarlyInit();

// Once boot homing is over: identity and the live link.
void initFleet();

// True in fleet builds, where the fleet server replaces the R&D cloud link.
bool fleetOwnsCloudLink();

// A remote asked this machine to update (go:update) while it is on its error
// screen, where the menu's update flow cannot run. Checks the dashboard and
// installs in the background, the same way a dashboard command does.
void fleetStartUpdateFromRemote();

// While that runs: the state to report to the remote in place of the real one
// ("update.checking", "update.installing", then "update.idle" or
// "update.failed" for a few seconds), so the remote's own update screens
// follow it. nullptr when nothing is running.
const char *fleetVirtualState();

// Free the live link's memory while another job needs it, and restore it.
void fleetPauseLink();
void fleetResumeLink();

#endif  // OSSM_SERVICES_FLEET_H
