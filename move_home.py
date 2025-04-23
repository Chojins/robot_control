#!/usr/bin/env python3

import os
import sys
import json
import time
import math
import numpy as np

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# Configuration for both robots
ROBOT_CONFIG = {
    "red": {
        "port": "/dev/ttyACM1",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/red_follower.json"),
        "home_file":        os.path.expanduser("~/lerobot/.cache/calibration/so100/red_home_position.json")
    },
    "blue": {
        "port": "/dev/ttyACM2",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_follower.json"),
        "home_file":        os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_home_position.json")
    }
}

# Hard‑coded preset angles
PRESET_POSITION = {
    "shoulder_pan": 2.0,
    "shoulder_lift": 163.0,
    "elbow_flex": 120.0,
    "wrist_flex": 78.0,
    "wrist_roll": -101.0,
    "gripper": 0.0
}

# Trajectory parameters
MAX_SPEED = 50.0   # deg/sec
ACCEL     = 80.0   # deg/sec^2
INTERVAL  = 0.02   # sec (20 ms control loop)


def bang_bang_trajectory(start, end, accel, max_speed, dt):
    delta     = end - start
    direction = np.sign(delta) if delta != 0 else 1
    dist      = abs(delta)
    if dist < 1e-6:
        return np.array([start]), np.array([0.0])

    t_a = max_speed / accel
    d_a = 0.5 * accel * t_a**2

    if 2*d_a < dist:
        # trapezoid profile
        d_c = dist - 2*d_a
        t_c = d_c / max_speed
        T   = 2*t_a + t_c
        v   = max_speed
    else:
        # triangular profile
        t_a = math.sqrt(dist / accel)
        v   = accel * t_a
        d_a = 0.5 * accel * t_a**2
        d_c = 0.0
        t_c = 0.0
        T   = 2*t_a

    steps = int(math.ceil(T / dt))
    times = np.linspace(0, T, steps)
    pos   = []
    for t in times:
        if t < t_a:
            x = 0.5 * accel * t**2
        elif t < (t_a + t_c):
            x = d_a + v * (t - t_a)
        else:
            td = t - t_a - t_c
            x = d_a + d_c + v * td - 0.5 * accel * td**2
        pos.append(start + direction * x)

    return np.array(pos), times


def synchronized_bang_bang(starts, ends, accel, max_speed, dt):
    """
    Generate bang‑bang profiles for multiple joints so they all finish together.
    """
    # compute distances
    dists = [abs(e - s) for s, e in zip(starts, ends)]

    # nominal duration for each joint at max_speed
    def nominal_T(dist):
        t_a_max = max_speed / accel
        d_a_max = 0.5 * accel * t_a_max**2
        if 2*d_a_max < dist:
            return 2*t_a_max + (dist - 2*d_a_max) / max_speed
        else:
            return 2 * math.sqrt(dist / accel)

    Ts = [nominal_T(d) for d in dists]
    T_all = max(Ts)

    # solve per‑joint peak velocity to match T_all
    v_peaks = []
    for d in dists:
        disc = (accel * T_all)**2 - 4 * accel * d
        v_pk = (accel * T_all - math.sqrt(max(0, disc))) / 2
        v_peaks.append(min(v_pk, max_speed))

    # generate each joint's profile
    profiles = []
    for s, e, v_pk in zip(starts, ends, v_peaks):
        pos, _ = bang_bang_trajectory(s, e, accel, v_pk, dt)
        profiles.append(pos)

    # pad to common length
    N = max(len(p) for p in profiles)
    for i, p in enumerate(profiles):
        if len(p) < N:
            profiles[i] = np.concatenate([p, np.full(N - len(p), p[-1])])

    times = np.linspace(0, T_all, N)
    return profiles, times


def run_synchronized(bus, motor_names, start, targets, move_name="MOVE"):
    """
    Stream synchronized bang‑bang profiles to the bus, starting from 'start' and ending at 'targets'.
    """
    profiles, times = synchronized_bang_bang(
        starts=list(start),
        ends=list(targets),
        accel=ACCEL,
        max_speed=MAX_SPEED,
        dt=INTERVAL
    )
    cmd = np.vstack(profiles).T  # shape: (steps, joints)
    print(f"→ Running '{move_name}' ({len(times)} steps)...")
    for step_cmd in cmd:
        bus.write("Goal_Position", step_cmd.tolist())
        time.sleep(INTERVAL)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ROBOT_CONFIG:
        print("Usage: python move_home.py [red|blue]")
        sys.exit(1)

    cfg = ROBOT_CONFIG[sys.argv[1]]
    port       = cfg["port"]
    cal_file   = cfg["calibration_file"]
    home_file  = cfg["home_file"]

    for f in (cal_file, home_file):
        if not os.path.isfile(f):
            print(f"Required file not found: {f}")
            sys.exit(1)

    # load home positions
    with open(home_file) as f:
        home_dict = json.load(f)
    motor_keys = list(home_dict.keys())

    # connect & calibrate
    bus_cfg = FeetechMotorsBusConfig(
        port=port,
        motors={name: [i+1, "sts3215"] for i, name in enumerate(motor_keys)}
    )
    bus = FeetechMotorsBus(bus_cfg)
    bus.connect()
    with open(cal_file) as f:
        bus.set_calibration(json.load(f))
    print(f"Connected & calibrated on {port}")

    motor_names = bus.motor_names

    # 1) synchronized → preset
    print("Moving to PRESET position (synchronized)...")
    start_pos = np.array(bus.read("Present_Position"))
    preset_targets = np.array([PRESET_POSITION[n] for n in motor_names])
    run_synchronized(bus, motor_names, start_pos, preset_targets, move_name="PRESET")

    final_preset = bus.read("Present_Position")
    print("Reached PRESET:")
    for name, angle in zip(motor_names, final_preset):
        print(f"  {name}: {angle:.2f}")

    # 2) synchronized → home
    print("Moving to HOME position (synchronized)...")
    start_pos = np.array(final_preset)
    home_targets = np.array([home_dict[n] for n in motor_names])
    run_synchronized(bus, motor_names, start_pos, home_targets, move_name="HOME")

    final_home = bus.read("Present_Position")
    print("Reached HOME:")
    for name, angle in zip(motor_names, final_home):
        print(f"  {name}: {angle:.2f}")

    bus.disconnect()
    print("Disconnected.")


if __name__ == "__main__":
    main()
