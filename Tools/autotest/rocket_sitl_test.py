#!/usr/bin/env python3
"""
Drive the SITL rocket model through a launch and report what the ROCKET
mode did.

Starts bin/arduplane with --model rocket, waits for the EKF, switches to
ROCKET, arms, fires the igniter on servo 9, then follows the RCKT log
messages and the attitude until the mode hands over.

Usage:
    rocket_sitl_test.py [--frame rocket] [--no-gps] [--speedup 10]
"""

import argparse
import os
import signal
import subprocess
import sys
import time

from pymavlink import mavutil

AUTOTEST_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.realpath(os.path.join(AUTOTEST_DIR, '..', '..'))
BINARY = os.path.join(ROOT_DIR, 'build', 'sitl', 'bin', 'arduplane')

# Phase numbers from ModeRocket::Phase
PHASES = {0: 'PAD', 1: 'RAIL', 2: 'BOOST', 3: 'COAST', 4: 'TRANS', 5: 'ABORT'}
MODE_ROCKET = 27


def wait_ekf_ready(mav, timeout=120):
    print('waiting for EKF')
    start = time.time()
    while time.time() - start < timeout:
        m = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if m is None:
            continue
        flags = m.flags
        attitude = flags & mavutil.mavlink.EKF_ATTITUDE
        horiz_vel = flags & mavutil.mavlink.EKF_VELOCITY_HORIZ
        if attitude and horiz_vel:
            print('EKF ready (flags=0x%x)' % flags)
            return True
    print('EKF never came ready')
    return False


def wait_ahrs_active(mav, timeout=60):
    """Wait until EKF3 is the active AHRS backend.

    Arming checks "AHRS: not using configured AHRS type" until the EKF has
    taken over from the DCM fallback, which happens a few seconds after the
    EKF_STATUS_REPORT flags already look healthy.
    """
    print('waiting for EKF3 to go active')
    start = time.time()
    while time.time() - start < timeout:
        m = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if m is not None:
            print('  AP: %s' % m.text)
            if 'EKF3 active' in m.text:
                return True
    # Not fatal: the arming attempt reports the real reason anyway.
    print('did not see "EKF3 active"')
    return False


def arm_and_launch(mav, args):
    # ROCKET mode first: _enter() latches the launch attitude.
    print('setting mode ROCKET (%d)' % MODE_ROCKET)
    mav.set_mode(MODE_ROCKET)
    time.sleep(1)

    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if hb is not None:
        print('custom_mode now %d' % hb.custom_mode)
        if hb.custom_mode != MODE_ROCKET:
            print('FAIL: mode did not change to ROCKET')
            return False

    if args.no_gps:
        # Take GPS away only once ROCKET mode is held, which is the real
        # sequence: the airframe is set up on the pad and then flies with
        # whatever it has. Bit 3 (8) skips the GPS lock check, bit 4 (16)
        # skips the INS group, where AP_Arming_Plane::ins_checks calls
        # AP::ahrs().pre_arm_check. A near-vertical airframe fails that
        # check's "EKF3 Roll/Pitch inconsistent" test, because the DCM
        # fallback cannot resolve an 85 degree pitch.
        print('disabling GPS')
        set_param(mav, 'ARMING_SKIPCHK', 24)
        set_param(mav, 'SIM_GPS1_ENABLE', 0)
        time.sleep(3)
        for _ in range(10):
            m = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None:
                print('  AP: %s' % m.text)
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb is not None:
            print('mode after GPS loss: %d' % hb.custom_mode)

    print('arming')
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 1, 0, 0, 0, 0, 0, 0)
    start = time.time()
    armed = False
    ack_result = None
    while time.time() - start < 15:
        m = mav.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'],
                           blocking=True, timeout=1)
        if m is None:
            if ack_result is not None:
                break
            continue
        if m.get_type() == 'STATUSTEXT':
            print('  AP: %s' % m.text)
        elif m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            ack_result = m.result
            armed = (m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
            print('arm result %d' % m.result)
            if armed:
                break
            # Keep reading briefly: the reason arrives as STATUSTEXT and
            # often lands just after the ack.
    if not armed:
        print('FAIL: could not arm')
        return False

    time.sleep(1)
    print('IGNITION: servo 9 -> 2000us')
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                              0, 9, 2000, 0, 0, 0, 0, 0)
    return True

def start_sitl(frame, speedup, workdir):
    cmd = [
        BINARY,
        '--model', frame,
        '--speedup', str(speedup),
        '--home', '-35.363261,149.165230,584,0',
        '--instance', '0',
    ]
    print('starting: %s' % ' '.join(cmd))
    return subprocess.Popen(cmd, cwd=workdir,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            bufsize=1,
                            preexec_fn=os.setsid)


def wait_heartbeat(mav, timeout=30):
    print('waiting for heartbeat')
    start = time.time()
    while time.time() - start < timeout:
        if mav.wait_heartbeat(timeout=1) is not None:
            print('got heartbeat')
            return True
    return False


def set_param(mav, name, value):
    mav.mav.param_set_send(mav.target_system, mav.target_component,
                           name.encode('utf-8'), float(value),
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    start = time.time()
    while time.time() - start < 5:
        m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if m is not None and m.param_id.rstrip('\x00') == name:
            return True
    print('WARNING: no confirmation for %s' % name)
    return False


def monitor_flight(mav, duration=40):
    """Follow the flight and collect what the mode actually did."""
    result = {
        'phases': [],
        'max_alt': 0.0,
        'max_speed': 0.0,
        'max_accel_g': 0.0,
        'final_mode': None,
        'aborted': False,
        'statustexts': [],
        'max_tilt_deg': 0.0,
        # SIMSTATE is the simulator's ground truth, independent of the EKF.
        'sim_max_alt': 0.0,
        'sim_max_tilt_deg': 0.0,
        'sim_max_accel_g': 0.0,
        'max_roll_deg': 0.0,
        'max_rollrate_dps': 0.0,
        'phase_speeds': [],
        # Set while messages keep arriving. A vehicle that panics goes quiet
        # without the process dying, so silence has to be a failure in its
        # own right rather than being read as "nothing more happened".
        'alive_at_end': False,
    }
    start = time.time()
    last_msg_time = start
    print('--- flight ---')
    while time.time() - start < duration:
        m = mav.recv_match(blocking=True, timeout=2)
        if m is None:
            if time.time() - last_msg_time > 5.0:
                print('  [%5.1fs] no telemetry for 5 s' % (time.time() - start))
                last_msg_time = time.time()
            continue
        last_msg_time = time.time()
        t = m.get_type()
        if t == 'STATUSTEXT':
            text = m.text
            result['statustexts'].append(text)
            # Elapsed time matters for reading these logs: it separates "the
            # observation window ended" from "the transition never fired".
            print('  [%5.1fs] AP: %s' % (time.time() - start, text))
            if 'Rocket:' in text:
                for name in PHASES.values():
                    if text.startswith('Rocket: %s' % name):
                        # do_abort sends "Rocket: ABORT!" and enter_phase then
                        # sends "Rocket: ABORT v=...", so a straight append
                        # records the phase twice. Only a real change counts.
                        if not result['phases'] or result['phases'][-1] != name:
                            result['phases'].append(name)
                        # "Rocket: COAST v=84.5" carries the mode's own
                        # velocity estimate at the transition.
                        if 'v=' in text:
                            try:
                                v = float(text.split('v=')[1].split()[0])
                                result['phase_speeds'].append((name, v))
                            except (ValueError, IndexError):
                                pass
            if 'ABORT' in text:
                result['aborted'] = True
        elif t == 'SIMSTATE':
            # Ground truth attitude and acceleration, independent of the EKF.
            # SIMSTATE carries no altitude, so the trajectory height is
            # printed by the model itself.
            tilt = abs(90.0 - abs(m.pitch * 57.2958))
            result['sim_max_tilt_deg'] = max(result['sim_max_tilt_deg'], tilt)
            g = ((m.xacc**2 + m.yacc**2 + m.zacc**2) ** 0.5) / 9.80665
            result['sim_max_accel_g'] = max(result['sim_max_accel_g'], g)
        elif t == 'GLOBAL_POSITION_INT':
            alt = m.relative_alt * 0.001
            result['max_alt'] = max(result['max_alt'], alt)
            speed = (m.vx**2 + m.vy**2 + m.vz**2) ** 0.5 * 0.01
            result['max_speed'] = max(result['max_speed'], speed)
        elif t == 'RAW_IMU':
            g = ((m.xacc**2 + m.yacc**2 + m.zacc**2) ** 0.5) / 1000.0
            result['max_accel_g'] = max(result['max_accel_g'], g)
        elif t == 'ATTITUDE':
            # Tilt away from the launch axis. Pitch is near +90 on the pad,
            # so the interesting number is how far it has fallen from vertical.
            tilt = abs(90.0 - abs(m.pitch * 57.2958))
            result['max_tilt_deg'] = max(result['max_tilt_deg'], tilt)
            result['max_roll_deg'] = max(result['max_roll_deg'],
                                         abs(m.roll * 57.2958))
            result['max_rollrate_dps'] = max(result['max_rollrate_dps'],
                                             abs(m.rollspeed * 57.2958))
        elif t == 'HEARTBEAT':
            if result['final_mode'] is not None and m.custom_mode != result['final_mode']:
                print('  [%5.1fs] mode %d -> %d'
                      % (time.time() - start, result['final_mode'], m.custom_mode))
            result['final_mode'] = m.custom_mode
    result['alive_at_end'] = (time.time() - last_msg_time) < 5.0
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', default='rocket',
                        help='rocket, rocket-vertical, rocket-tilted, '
                             'rocket-nofins, rocket-x45')
    parser.add_argument('--speedup', type=int, default=5)
    parser.add_argument('--no-gps', action='store_true',
                        help='fly on inertial velocity only')
    parser.add_argument('--duration', type=int, default=40)
    parser.add_argument('--set', action='append', default=[],
                        metavar='NAME=VALUE',
                        help='override a parameter before launch, repeatable')
    args = parser.parse_args()

    if not os.path.exists(BINARY):
        print('no binary at %s, run ./waf plane first' % BINARY)
        return 1

    workdir = os.path.join(AUTOTEST_DIR, 'rocket_sitl_run')
    os.makedirs(workdir, exist_ok=True)
    # SITL persists parameters in eeprom.bin in the working directory, so a
    # run that changed EK3_SRC or ARMING_SKIPCHK would otherwise poison every
    # later run. Each case starts from the model defaults.
    eeprom = os.path.join(workdir, 'eeprom.bin')
    if os.path.exists(eeprom):
        os.unlink(eeprom)

    sitl = start_sitl(args.frame, args.speedup, workdir)
    mav = None
    try:
        time.sleep(3)
        mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760',
                                         robust_parsing=True)
        if not wait_heartbeat(mav):
            print('FAIL: no heartbeat')
            return 1

        mav.mav.request_data_stream_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

        # The EKF has to settle on GPS before arming either way. In the
        # GPS-denied case it is taken away later, in arm_and_launch, once
        # ROCKET mode is held: starting GPS-denied leaves the EKF without an
        # origin and it never arms at all.
        wait_ekf_ready(mav, timeout=60)
        wait_ahrs_active(mav, timeout=30)

        for override in args.set:
            if '=' not in override:
                print('ignoring malformed --set %r' % override)
                continue
            name, value = override.split('=', 1)
            print('setting %s = %s' % (name, value))
            set_param(mav, name.strip(), float(value))

        if not arm_and_launch(mav, args):
            return 1
        result = monitor_flight(mav, args.duration)

        print('')
        print('=== result: %s%s ===' % (args.frame,
                                        ' (no GPS)' if args.no_gps else ''))
        print('phases reached : %s' % ' -> '.join(result['phases']))
        if result['phase_speeds']:
            print('mode speed est : %s' % ', '.join(
                '%s=%.1f' % (n, v) for n, v in result['phase_speeds']))
        print('max altitude   : %.1f m (EKF)' % result['max_alt'])
        print('max speed      : %.1f m/s (EKF)' % result['max_speed'])
        print('max accel      : %.1f g (IMU), %.1f g (true)'
              % (result['max_accel_g'], result['sim_max_accel_g']))
        print('max tilt       : %.1f deg (EKF), %.1f deg (true)'
              % (result['max_tilt_deg'], result['sim_max_tilt_deg']))
        print('max roll       : %.1f deg, rate %.0f deg/s'
              % (result['max_roll_deg'], result['max_rollrate_dps']))
        print('aborted        : %s' % result['aborted'])
        print('final mode     : %s' % result['final_mode'])

        reached = set(result['phases'])
        # The handover has to be checked, not just the phase sequence. An
        # earlier version passed on "reached TRANS" alone, which hid a
        # vehicle that entered TRANSITION and then wedged on the fixed-wing
        # rate controller panic: the phase was logged, the mode never
        # changed, and the test still said PASS.
        handed_over = (result['final_mode'] is not None
                       and result['final_mode'] != MODE_ROCKET)
        checks = [
            ('reached BOOST', 'BOOST' in reached),
            ('reached COAST or aborted',
             'COAST' in reached or result['aborted']),
            ('left ROCKET mode', handed_over or result['aborted']),
            ('telemetry alive at end', result['alive_at_end']),
        ]
        for name, passed in checks:
            print('check          : %-24s %s' % (name, 'ok' if passed else 'FAILED'))
        ok = all(passed for _, passed in checks)
        print('verdict        : %s' % ('PASS' if ok else 'FAIL'))
        return 0 if ok else 1
    finally:
        if mav is not None:
            mav.close()
        # Whether SITL was still alive matters: a mode change that crashes
        # the binary looks exactly like a transition that never fired, since
        # both end with the telemetry going quiet.
        died_early = sitl.poll() is not None
        if not died_early:
            os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
            try:
                sitl.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
        out = sitl.stdout.read() if sitl.stdout else ''
        for line in out.splitlines():
            if 'Rocket:' in line or 'ground' in line:
                print('  SIM: %s' % line)
        if died_early:
            print('SITL EXITED ON ITS OWN, status %s' % sitl.returncode)
            print('--- last 25 lines of SITL output ---')
            for line in out.splitlines()[-25:]:
                print('  %s' % line)


if __name__ == '__main__':
    sys.exit(main())
