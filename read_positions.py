#!/usr/bin/env python3

import json
import os
import sys

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# Configuration for both robots
ROBOT_CONFIG = {
    "red": {
        "port": "/dev/ttyACM1",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/red_follower.json"),
        "output_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/red_home_position.json")
    },
    "blue": {
        "port": "/dev/ttyACM2",
        "calibration_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_follower.json"),
        "output_file": os.path.expanduser("~/lerobot/.cache/calibration/so100/blue_home_position.json")
    }
}

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ROBOT_CONFIG:
        print("Usage: python read_positions.py [red|blue] [--save]")
        sys.exit(1)

    robot_name = sys.argv[1]
    save_to_file = "--save" in sys.argv

    robot = ROBOT_CONFIG[robot_name]
    port = robot["port"]
    calibration_file = robot["calibration_file"]
    output_file = robot["output_file"]

    # Configure the Feetech bus
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
    print(f"Connected to {robot_name} robot on {port}")

    # Load calibration if available
    if os.path.exists(calibration_file):
        with open(calibration_file, "r") as f:
            calibration_data = json.load(f)
        motors_bus.set_calibration(calibration_data)
        print("Calibration loaded.")
    else:
        print(f"No calibration file found at {calibration_file}. Reading raw motor steps instead of degrees.")

    # Read motor positions
    positions = motors_bus.read("Present_Position")
    motor_names = motors_bus.motor_names

    # Print them
    print("Current motor positions:")
    for name, pos in zip(motor_names, positions):
        print(f"  {name}: {pos:.2f}")

    # Optionally save
    if save_to_file:
        position_dict = {name: float(f"{pos:.2f}") for name, pos in zip(motor_names, positions)}
        with open(output_file, "w") as f:
            json.dump(position_dict, f, indent=2)
        print(f"Saved current positions to {output_file}")

    motors_bus.disconnect()
    print("Disconnected.")

if __name__ == "__main__":
    main()
