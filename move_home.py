#!/usr/bin/env python3

import os
import sys
import json
import time
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
    "shoulder_pan": 0.0,
    "shoulder_lift": 163.0,
    "elbow_flex": 120.0,
    "wrist_flex": 78.0,
    "wrist_roll": -101.0,
    "gripper": 0.0
}

# Trajectory parameters
MAX_SPEED = 40.0        # deg/sec
ACCEL     = 60.0       # deg/sec^2
INTERVAL  = 0.02        # control loop interval (s)

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
        t_a = np.sqrt(dist / accel)
        v   = accel * t_a
        d_a = 0.5 * accel * t_a**2
        d_c = 0.0     # <— ensure it's set!
        t_c = 0.0
        T   = 2*t_a

    steps = int(np.ceil(T / dt))
    times = np.linspace(0, T, steps)
    pos   = []
    for t in times:
        if t < t_a:
            x = 0.5 * accel * t**2
        elif t < (t_a + t_c):
            x = d_a + v * (t - t_a)
        else:
            td = t - t_a - t_c
            # <-- corrected deceleration
            x = d_a + d_c + v*td - 0.5*accel*td**2
        pos.append(start + direction * x)

    return np.array(pos), times


def run_trajectory(bus, motor_names, start, targets):
    """
    Given start and target arrays, compute per-joint bang‑bang trajectories
    and stream them to the robot bus.
    """
    # build each joint's profile
    profiles = []
    for i in range(len(motor_names)):
        traj, _ = bang_bang_trajectory(
            start[i], targets[i],
            accel=ACCEL, max_speed=MAX_SPEED, dt=INTERVAL
        )
        profiles.append(traj)

    # pad them all to the same length
    length = max(len(p) for p in profiles)
    for i, p in enumerate(profiles):
        if len(p) < length:
            pad = np.full(length - len(p), p[-1])
            profiles[i] = np.concatenate([p, pad])

    # stream to bus
    for step in range(length):
        bus.write("Goal_Position", [float(profiles[i][step]) for i in range(len(motor_names))])
        time.sleep(INTERVAL)

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ROBOT_CONFIG:
        print("Usage: python move_home.py [red|blue]")
        sys.exit(1)

    cfg = ROBOT_CONFIG[sys.argv[1]]
    port = cfg["port"]
    cal_file  = cfg["calibration_file"]
    home_file = cfg["home_file"]

    for f in (cal_file, home_file):
        if not os.path.isfile(f):
            print(f"Required file not found: {f}")
            sys.exit(1)

    # Load home positions
    with open(home_file) as f:
        home_dict = json.load(f)
    print(f"Loaded home positions from {home_file}")

    # Connect & calibrate
    bus_cfg = FeetechMotorsBusConfig(
        port=port,
        motors={
            name: [i+1, "sts3215"]
            for i, name in enumerate(home_dict.keys())
        }
    )
    bus = FeetechMotorsBus(bus_cfg)
    bus.connect()
    with open(cal_file) as f:
        bus.set_calibration(json.load(f))
    print(f"Connected & calibrated on {port}")

    motor_names = bus.motor_names

    # 1) Trajectory → preset
    print("Moving to PRESET position via bang-bang...")
    start_pos = np.array(bus.read("Present_Position"))
    preset_targets = np.array([PRESET_POSITION[n] for n in motor_names])
    run_trajectory(bus, motor_names, start_pos, preset_targets)

    final_preset = bus.read("Present_Position")
    print("Reached PRESET:")
    for n, p in zip(motor_names, final_preset):
        print(f"  {n}: {p:.2f}")

    # 2) Trajectory → home
    print("Moving to HOME position via bang-bang...")
    start_pos = np.array(final_preset)
    home_targets = np.array([home_dict[n] for n in motor_names])
    run_trajectory(bus, motor_names, start_pos, home_targets)

    final_home = bus.read("Present_Position")
    print("Reached HOME:")
    for n, p in zip(motor_names, final_home):
        print(f"  {n}: {p:.2f}")

    bus.disconnect()
    print("Disconnected.")

if __name__ == "__main__":
    main()
