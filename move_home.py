#!/usr/bin/env python3

import os
import sys
import json
import time

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# Configuration for both robots
ROBOT_CONFIG = {
    "red": {
        "port": "/dev/ttyACM1",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/red_follower.json"),
        "home_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/red_home_position.json")
    },
    "blue": {
        "port": "/dev/ttyACM2",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_follower.json"),
        "home_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_home_position.json")
    }
}

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ROBOT_CONFIG:
        print("Usage: python move_home.py [red|blue]")
        sys.exit(1)

    robot_name = sys.argv[1]
    robot = ROBOT_CONFIG[robot_name]

    port = robot["port"]
    calibration_file = robot["calibration_file"]
    home_file = robot["home_file"]

    if not os.path.isfile(calibration_file):
        print(f"Calibration file not found at: {calibration_file}")
        print("Exiting without moving the robot.")
        return

    if not os.path.isfile(home_file):
        print(f"Home position file not found at: {home_file}")
        print("Run 'read_positions.py [red|blue] --save' to create it.")
        return

    # Load home position from file
    with open(home_file, "r") as f:
        desired_positions = json.load(f)
    print(f"Loaded home positions from {home_file}")

    # Configure motor bus
    bus_config = FeetechMotorsBusConfig(
        port=port,
        motors={
            "shoulder_pan":  [1, "sts3215"],
            "shoulder_lift": [2, "sts3215"],
            "elbow_flex":    [3, "sts3215"],
            "wrist_flex":    [4, "sts3215"],
            "wrist_roll":    [5, "sts3215"],
            "gripper":       [6, "sts3215"],
        },
    )

    motors_bus = FeetechMotorsBus(bus_config)
    motors_bus.connect()
    print(f"Connected to {robot_name} robot on port {port}")

    with open(calibration_file, "r") as f:
        calibration_data = json.load(f)
    motors_bus.set_calibration(calibration_data)
    print("Calibration loaded.")

    motor_names = motors_bus.motor_names
    target_list = [desired_positions[name] for name in motor_names]

    motors_bus.write("Goal_Position", target_list)
    print("Moving to home position...")

    time.sleep(2.0)

    final_positions = motors_bus.read("Present_Position")
    print("Final positions:")
    for name, pos in zip(motor_names, final_positions):
        print(f"  {name}: {pos:.2f}")

    motors_bus.disconnect()
    print("Disconnected.")

if __name__ == "__main__":
    main()
