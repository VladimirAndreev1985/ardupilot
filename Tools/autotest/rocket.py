"""
SITL smoke test for the ROCKET flight mode (RD1-300 + X-45 tail).

The test:
  1. arms the vehicle in MANUAL mode
  2. switches to ROCKET (mode 27)
  3. injects a fake launch via SIM_ACC_TRIM / SITL accel overlay
  4. waits for stage transition BOOST -> COAST -> DONE -> POST_MODE
  5. verifies that fin PWM outputs move during the active phase

Run with:
  Tools/autotest/autotest.py fly.ArduPlane.Rocket
"""

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module  # noqa: F401  placeholder


def rocket_state_name(state):
    return {
        0: "WAIT_LAUNCH",
        1: "BOOST",
        2: "COAST",
        3: "DONE",
    }.get(state, "UNKNOWN(%d)" % state)


# Intentionally left as a library: the actual test is wired in
# Tools/autotest/arduplane.py so that it inherits from AutoTestPlane
# and matches the existing test framework conventions.
