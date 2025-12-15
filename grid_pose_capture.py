"""
Interactive grid pose capture utility.

Records joint poses for each named grid cell (R#C#), along with the gripper
state to use before moving into that cell and the gripper state once the arm is
positioned (e.g., open vs. holding). Also lets you capture a default pose
outside the grid.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from so101_control import (
    PORT,
    connect_robot,
    disable_torque_all,
    enable_torque_all,
)

JOINT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

DEFAULT_OUTPUT = Path("grid_pose_map.json")


def prompt_float(prompt: str, default: float | None = None) -> float:
    while True:
        raw = input(f"{prompt} [{default if default is not None else ''}]: ").strip()
        if not raw:
            if default is not None:
                return float(default)
            continue
        try:
            return float(raw)
        except ValueError:
            print("Invalid number, try again.")


def prompt_str(prompt: str) -> str:
    while True:
        raw = input(prompt).strip()
        if raw:
            return raw
        print("Value required.")


def capture_pose(robot, label: str) -> dict:
    print(f"\n--- Capturing pose for '{label}' ---")
    pre_ticks = prompt_float("Gripper ticks BEFORE moving toward this pose", 3315.0)
    hold_ticks = prompt_float("Gripper ticks WHILE at this pose", 2176.0)

    try:
        disable_torque_all(robot)
    except Exception as exc:
        print(f"⚠️  Warning: could not disable torque ({exc})")
    input("Torque disabled. Manually move the arm to the desired pose, then press ENTER...")
    enable_torque_all(robot)
    time.sleep(0.3)

    obs = robot.get_observation()
    joints = {key: float(obs.get(key, 0.0)) for key in JOINT_KEYS}
    print("Captured joints:")
    for key, val in joints.items():
        print(f"  {key:18s}: {val:8.2f}")

    return {
        "label": label,
        "joints": joints,
        "pre_gripper_ticks": pre_ticks,
        "grid_gripper_ticks": hold_ticks,
    }


def main() -> None:
    print("=" * 60)
    print("GRID POSE CAPTURE UTILITY")
    print("=" * 60)
    print(f"Connecting to robot on {PORT} ...")
    robot = connect_robot(PORT)
    print("✓ Connected.")

    data = {
        "default_pose": None,
        "grids": {},
    }
    if DEFAULT_OUTPUT.exists():
        try:
            existing = json.loads(DEFAULT_OUTPUT.read_text())
            if isinstance(existing, dict):
                data.update(existing)
        except json.JSONDecodeError:
            pass

    menu = (
        "\nOptions:\n"
        "  [g] Capture grid cell pose\n"
        "  [d] Capture default (home/off-grid) pose\n"
        "  [l] List stored entries\n"
        "  [s] Save & exit\n"
        "  [q] Quit without saving\n"
    )

    try:
        while True:
            print(menu)
            choice = input("Select option: ").strip().lower()
            if choice == "g":
                label = prompt_str("Grid label (e.g., R2C3): ")
                entry = capture_pose(robot, label)
                data["grids"][label] = entry
                print(f"✓ Stored pose for {label}")
            elif choice == "d":
                entry = capture_pose(robot, "default")
                data["default_pose"] = entry
                print("✓ Stored default pose")
            elif choice == "l":
                print("\nStored entries:")
                if data.get("default_pose"):
                    print("  Default pose recorded.")
                for label in sorted(data["grids"]):
                    print(f"  {label}")
            elif choice == "s":
                DEFAULT_OUTPUT.write_text(json.dumps(data, indent=2))
                print(f"✓ Saved to {DEFAULT_OUTPUT}")
                break
            elif choice == "q":
                print("Exiting without saving.")
                break
            else:
                print("Unknown option.")
    finally:
        print("\nDisconnecting...")
        robot.disconnect()
        print("✓ Disconnected.")


if __name__ == "__main__":
    main()
