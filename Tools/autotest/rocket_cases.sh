#!/bin/bash
# Run the six ROCKET mode cases that matter: nominal, GPS-denied, an angled
# rail departure, zero fin authority, the X-45 fin layout, and an abort.
# Gains come from models/rocket.parm, which carries the values SITL showed
# to be stable.
#
# Duration is 25 s because apogee detection by climb rate hands over later
# than the old speed test did: the rocket now stays in COAST until the
# vertical velocity actually reaches RKT_TRANS_VZ, around 13 s after launch.
set -u

cd "$(dirname "$0")/../.." || exit 1

echo "########## 1. nominal ##########"
timeout 200 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket --duration 25 2>&1 | tail -30

echo ""
echo "########## 2. GPS-denied ##########"
timeout 250 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket --no-gps --duration 25 2>&1 | tail -30

echo ""
echo "########## 3. tilted rail (30 deg off vertical) ##########"
timeout 200 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket-tilted --duration 25 2>&1 | tail -30

echo ""
echo "########## 4. no fin authority ##########"
timeout 200 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket-nofins --duration 25 2>&1 | tail -30

echo ""
echo "########## 5. X-45 fin layout ##########"
timeout 200 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket-x45 --duration 25 2>&1 | tail -30

# The tilted-rail case used to be the abort case, but it now flies an arc to
# apogee and hands over, which is the correct behaviour: an angled trajectory
# is not a failure. That left the abort path untested, so it gets its own
# case. It uses the COAST rate limit rather than the attitude limit: the
# attitude test measures deviation from the launch attitude, and the
# simulated rocket tracks its rail so closely that even a 1 deg limit does
# not trip during boost. Tightening the tumble rate to 20 deg/s does fire,
# at about 2.6 s, just after burnout, which is the COAST branch of
# should_abort plus do_abort and resolve_post_mode.
echo ""
echo "########## 6. abort on COAST tumble rate ##########"
timeout 200 python3 -u Tools/autotest/rocket_sitl_test.py \
    --frame rocket --duration 25 \
    --set RKT_ABORT_RATE=20 2>&1 | tail -30
