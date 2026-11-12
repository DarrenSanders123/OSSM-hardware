# Adds the fleet server settings to the `fleet` build.
#
# FLEET_SERVER  base address of your fleet server, e.g. https://ossm.example.com
# FLEET_KEY     the FLEET_DEVICE_KEY from that server's .env
#
# Both come from environment variables (used by the GitHub workflow) or from a
# `fleet.env` file next to platformio.ini, which is git-ignored.
Import("env")

import os
import sys

values = {}
env_file = os.path.join(env["PROJECT_DIR"], "fleet.env")
if os.path.isfile(env_file):
    with open(env_file) as handle:
        for line in handle:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip().strip('"').strip("'")

server = os.getenv("FLEET_SERVER", values.get("FLEET_SERVER", "")).rstrip("/")
key = os.getenv("FLEET_KEY", values.get("FLEET_KEY", ""))

if not server.startswith("https://") or not key:
    sys.stderr.write(
        "fleet build: set FLEET_SERVER (https://...) and FLEET_KEY, either as "
        "environment variables or in Software/fleet.env\n"
    )
    env.Exit(1)

env.Append(
    CPPDEFINES=[
        ("FLEET_SERVER", env.StringifyMacro(server)),
        ("FLEET_KEY", env.StringifyMacro(key)),
        ("FLEET_ARTIFACT_PREFIX", env.StringifyMacro(server + "/firmware/releases/")),
    ]
)
