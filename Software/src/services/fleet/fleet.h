#ifndef OSSM_SERVICES_FLEET_H
#define OSSM_SERVICES_FLEET_H

/**
 * Fleet client: lets the builder's own server restart this machine or tell it
 * to update, without the owner touching it.
 *
 * Only active in builds that define FLEET_SERVER and FLEET_KEY (the `fleet`
 * PlatformIO environment); otherwise initFleet() does nothing.
 *
 * Safety: the machine only talks to the fleet server, and only acts on a
 * command, while it is idle in the menu or stopped on the error screen. It
 * never checks in, restarts or updates while a session is running.
 */
void initFleet();

#endif  // OSSM_SERVICES_FLEET_H
