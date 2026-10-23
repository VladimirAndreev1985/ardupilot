#!/bin/bash
# Sweep the ROCKET roll gains in SITL and report what each pair does.
# Each case starts from a clean eeprom, so the values under test are the
# only thing that differs. Run from the ardupilot root with the venv active.
set -u

cd "$(dirname "$0")/../.." || exit 1

for pair in "$@"; do
    kp="${pair%%:*}"
    kd="${pair##*:}"
    echo "##### RKT_R_KP=${kp} RKT_R_KD=${kd} #####"
    timeout 150 python3 -u Tools/autotest/rocket_sitl_test.py \
        --frame rocket --duration 16 \
        --set "RKT_R_KP=${kp}" --set "RKT_R_KD=${kd}" 2>&1 \
        | grep -E "max roll|max tilt|phases|verdict|aborted|apogee"
done
