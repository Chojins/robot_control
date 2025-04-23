#!/usr/bin/env python3
import os
import sys
import json

from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, CalibrationMode

HALF_TURN_DEGREE = 180.0

# ——————————————————————————————————————————————————————————————
# 1) Per‑arm file locations and leader setup
# ——————————————————————————————————————————————————————————————
FOLLOWER_CFG = {
    "red": {
        "port":     "/dev/ttyACM1",
        "cal_file": os.path.expanduser("~/lerobot/.cache/calibration/so100-red/main_follower.json")
    },
    "blue": {
        "port":     "/dev/ttyACM2",
        "cal_file": os.path.expanduser("~/lerobot/.cache/calibration/so100-blue/main_follower.json")
    }
}

LEADER_CFG = {
    # single physical leader arm 
    "port": "/dev/ttyACM0",
    "cal_files": [
        os.path.expanduser("~/lerobot/.cache/calibration/so100-red/main_leader.json"),
        os.path.expanduser("~/lerobot/.cache/calibration/so100-blue/main_leader.json")
    ]
}


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def save_json(obj, path):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    print(f"✅ Saved updated calibration to {path}")


def compute_new_offsets(cal, raw_ticks, known_angles, model_res):
    new_offsets = cal["homing_offset"].copy()
    for i, name in enumerate(cal["motor_names"]):
        if CalibrationMode[cal["calib_mode"][i]] != CalibrationMode.DEGREE:
            continue

        drive = cal["drive_mode"][i]
        sign  = -1 if drive else +1
        res   = model_res[name]
        r     = raw_ticks[i]
        θ_k   = known_angles[name]

        # invert: θ_k = ((sign*r + h_new)/(res/2))*180
        h_new = θ_k * (res/2) / HALF_TURN_DEGREE - sign * r
        new_offsets[i] = int(round(h_new))

    return new_offsets


def calibrate_and_read(bus, cal_data):
    bus.set_calibration(cal_data)
    return bus.read("Present_Position")


def process_file(label, bus, cal_path, raw_ticks, known_angles, model_res):
    basename = os.path.basename(cal_path)
    print(f"\n=== Calibrating {label} @ {basename} ===")

    orig_cal  = load_json(cal_path)

    # show old angles
    old_angles = calibrate_and_read(bus, orig_cal)
    print(" Old calibrated angles (°):")
    for n,a in zip(orig_cal["motor_names"], old_angles):
        print(f"  {n:15s}: {a:.2f}")

    # compute proposals
    new_offsets = compute_new_offsets(orig_cal, raw_ticks, known_angles, model_res)
    print("\n Proposed homing_offset changes:")
    for i,n in enumerate(orig_cal["motor_names"]):
        old_h, new_h = orig_cal["homing_offset"][i], new_offsets[i]
        if old_h != new_h:
            print(f"  {n:15s}: {old_h:6d} → {new_h:6d}   (Δ {new_h-old_h:+d})")

    # show new angles
    cand_cal = orig_cal.copy()
    cand_cal["homing_offset"] = new_offsets
    new_angles = calibrate_and_read(bus, cand_cal)
    print("\n New calibrated angles (°):")
    for n,a in zip(orig_cal["motor_names"], new_angles):
        print(f"  {n:15s}: {a:.2f}")

    # prompt
    ans = input("\nSave these offsets? (y/N) ").strip().lower()
    if ans == "y":
        orig_cal["homing_offset"] = new_offsets
        save_json(orig_cal, cal_path)
    else:
        print("⏹  Skipped saving.")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("red", "blue", "leader"):
        print("Usage: python adjust_calibration_interactive.py [red|blue|leader] known_angles.json")
        sys.exit(1)

    mode         = sys.argv[1]
    known_angles = load_json(sys.argv[2])

    if mode in ("red", "blue"):
        # only follower file
        info = FOLLOWER_CFG[mode]
        # connect
        cfg = FeetechMotorsBusConfig(
            port=info["port"],
            motors={name: [i+1, "sts3215"]
                    for i,name in enumerate(load_json(info["cal_file"])["motor_names"])}
        )
        bus = FeetechMotorsBus(cfg)
        bus.connect()
        print(f"🔌 Connected (raw mode) on {info['port']}")

        raw_ticks = bus.read("Present_Position")
        model_res = { name: bus.model_resolution[cfg.motors[name][1]]
                      for name in load_json(info["cal_file"])["motor_names"] }

        process_file(f"{mode} follower arm", bus, info["cal_file"], raw_ticks, known_angles, model_res)
        bus.disconnect()

    else:  # leader mode
        info = LEADER_CFG
        # single connection on leader port
        cfg = FeetechMotorsBusConfig(
            port=info["port"],
            motors={name: [i+1, "sts3215"]
                    for i,name in enumerate(load_json(info["cal_files"][0])["motor_names"])}
        )
        bus = FeetechMotorsBus(cfg)
        bus.connect()
        print(f"🔌 Connected (raw mode) on {info['port']}")

        input("⏯  Position the leader arm at your known pose and hit Enter…")
        raw_ticks = bus.read("Present_Position")
        print(" Raw ticks:", dict(zip(load_json(info['cal_files'][0])["motor_names"], raw_ticks)))

        model_res = { name: bus.model_resolution[cfg.motors[name][1]]
                      for name in load_json(info["cal_files"][0])["motor_names"] }

        # update both leader files with same offsets
        for cal_path in info["cal_files"]:
            process_file("Leader arm", bus, cal_path, raw_ticks, known_angles, model_res)

        bus.disconnect()
        print("\n🏁 All done.")
