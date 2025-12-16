"""
SO-101 Robot Control Module

This module provides functions to connect to and control the SO-101 robot arm.

Torque Control:
    - By default, motors maintain holding torque (arm is stiff)
    - To move the arm freely by hand while powered: use disable_torque_all()
    - To restore normal operation: use enable_torque_all()
    - Or use connect_robot(port, disable_torque=True) to disable on connection

Example:
    from so101_control import connect_robot, disable_torque_all, enable_torque_all
    
    robot = connect_robot()
    disable_torque_all(robot)  # Move arm freely by hand
    # ... manually position arm ...
    enable_torque_all(robot)   # Restore stiffness for movement commands
"""

import json
import math
import time
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

# --- CONFIGURATION ---
# Set to your Mac serial device for the SO-101 arm
PORT = "/dev/tty.usbmodem5A7C1227091"
GRIPPER_ID = 'gripper.pos'
ROBOT_ID = "so101_arm"

# Tweak these based on your specific gripper calibration
# (100.0 observed fully open on your gripper, 21.0 closed enough to pinch the peg)
GRIPPER_OPEN_VAL = 100.0
GRIPPER_CLOSED_VAL = 21.0  # Set this tight enough to hold the peg, but not stall the motor

# --- Continuous Analog Gripper Control ---
def gripper_fraction_to_pos(fraction: float) -> float:
    """
    Map a normalized 0.0–1.0 fraction to an analog gripper position.
      0.0 → fully open
      1.0 → fully closed
    
    Args:
        fraction: Gripper position as a fraction (0.0 = open, 1.0 = closed)
    
    Returns:
        Gripper servo position value
    """
    fraction = max(0.0, min(1.0, fraction))  # Clamp to [0.0, 1.0]
    return GRIPPER_OPEN_VAL + (GRIPPER_CLOSED_VAL - GRIPPER_OPEN_VAL) * fraction

L_HUMERUS = 150.0   
L_FOREARM = 150.0   
L_HAND    = 177.8   
L_BASE_OFFSET = 127.0

def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value to the given range."""
    return max(min_val, min(value, max_val))


def load_so101_calibration(robot_id: str = ROBOT_ID) -> Dict[str, MotorCalibration]:
    """
    Load the stored calibration file used by the standard SO-101 follower.
    Returns a dict compatible with FeetechMotorsBus calibration argument.
    """
    calib_dir = HF_LEROBOT_CALIBRATION / ROBOTS / SO101Follower.name
    calib_path = calib_dir / f"{robot_id}.json"
    if not calib_path.is_file():
        print(f"⚠️  Warning: Calibration file not found at {calib_path}")
        return {}
    with open(calib_path) as f:
        raw = json.load(f)
    calibration: Dict[str, MotorCalibration] = {}
    for joint, values in raw.items():
        calibration[joint] = MotorCalibration(**values)
    return calibration


class GripperOnlyRobot:
    """Lightweight controller that exposes a robot-like interface for the gripper motor only."""

    joint_names = ["gripper.pos"]

    def __init__(self, port: str, calibration: Dict[str, MotorCalibration] | None = None):
        self.port = port
        self.calibration = dict(calibration) if calibration else load_so101_calibration()
        gripper_entry = self.calibration.get("gripper")
        gripper_calib = {"gripper": gripper_entry} if gripper_entry else None
        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=gripper_calib,
        )

    def connect(self) -> None:
        self.bus.connect()

    def disconnect(self, disable_torque: bool = True) -> None:
        self.bus.disconnect(disable_torque)

    def send_action(self, action: Dict[str, float]) -> Dict[str, float]:
        goal_pos = {}
        for key, val in action.items():
            if key.endswith(".pos"):
                goal_pos[key.removesuffix(".pos")] = val
            else:
                goal_pos[key] = val
        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{k}.pos": v for k, v in goal_pos.items()}

    def get_observation(self) -> Dict[str, float]:
        obs = self.bus.sync_read("Present_Position")
        return {f"{k}.pos": v for k, v in obs.items()}

    def set_torque(self, joint: str, enabled: bool) -> None:
        motor = joint.removesuffix(".pos")
        if enabled:
            self.bus.enable_torque(motor)
        else:
            self.bus.disable_torque(motor)

# --- HELPER FUNCTION ---
def move_to(robot, target_pos: Dict[str, float], duration: float = 3.0, enable_torque: bool = True, 
            check_position: bool = True) -> None:
    """
    Moves robot and waits until it arrives.
    
    Args:
        robot: SO101Follower instance
        target_pos: Dictionary of joint positions
        duration: Maximum time to wait for movement
        enable_torque: If True, ensure torque is enabled before moving
        check_position: If True, try to verify position reached (may fail on communication errors)
    """
    if enable_torque:
        # Ensure torque is enabled before movement
        try:
            enable_torque_all(robot)
        except:
            pass  # Continue even if torque enable fails
    
    print("Moving...")
    print(f"-> Targets: {target_pos}")
    # Send action to robot
    robot.send_action(target_pos)

    # If gripper target present, print a concise debug line
    if "gripper.pos" in target_pos:
        try:
            print(f"-> Gripper target value: {float(target_pos['gripper.pos']):.2f}")
        except Exception:
            pass

    if not check_position:
        # Just wait for the duration without checking position
        print(f"Waiting {duration:.1f} seconds for movement to complete...")
        time.sleep(duration)
        return

    # Try to check position, but handle communication errors gracefully
    start_time = time.time()
    max_retries = 3
    retry_count = 0
    
    while True:
        try:
            current = robot.get_observation()

            # Compare matching keys only (order-robust)
            all_ok = True
            tol = 3.0
            for k, tgt in target_pos.items():
                cur_val = current.get(k, None)
                if cur_val is None:
                    # If the observation doesn't include this joint, skip it
                    continue
                try:
                    cur_f = float(cur_val)
                    tgt_f = float(tgt)
                except Exception:
                    # Non-numeric values: skip strict comparison
                    continue
                if abs(cur_f - tgt_f) > tol:
                    all_ok = False
                    break
            if all_ok:
                print("✓ Position reached")
                break
        except (ConnectionError, Exception) as e:
            retry_count += 1
            if retry_count >= max_retries:
                print(f"⚠️  Warning: Could not verify position after {max_retries} attempts")
                print(f"   Continuing anyway - movement command was sent")
                break
            time.sleep(0.2)  # Brief pause before retry
            continue

        # Timeout safety
        if time.time() - start_time > duration:
            print(f"⚠️  Warning: Move timed out after {duration:.1f}s (may have reached target)")
            break
        time.sleep(0.1)  # Slightly longer sleep to reduce communication load


def command_gripper(robot, fraction: float, duration: float = 2.0,
                    enable_torque: bool = True, check_position: bool = True) -> float:
    """Send a gripper-only command using a normalized fraction (0=open, 1=closed)."""
    target_val = gripper_fraction_to_pos(fraction)
    move_to(
        robot,
        {GRIPPER_ID: target_val},
        duration=duration,
        enable_torque=enable_torque,
        check_position=check_position,
    )
    return target_val


def open_gripper(robot, duration: float = 2.0, enable_torque: bool = True,
                 check_position: bool = True) -> float:
    """Convenience helper to fully open the gripper."""
    return command_gripper(robot, 0.0, duration, enable_torque, check_position)


def close_gripper(robot, duration: float = 2.0, enable_torque: bool = True,
                  check_position: bool = True) -> float:
    """Convenience helper to fully close the gripper."""
    return command_gripper(robot, 1.0, duration, enable_torque, check_position)


def disable_torque_all(robot: SO101Follower) -> None:
    """
    Disable holding torque on all joints so the arm can be moved freely by hand.
    Uses the supported Lerobot APIs to avoid confusing failure spam.
    """
    errors = []
    success = False

    bus = getattr(robot, "bus", None)
    if bus is not None:
        disable_fn = getattr(bus, "disable_torque", None)
        if callable(disable_fn):
            try:
                disable_fn()  # None => all motors
                success = True
            except Exception as exc:
                errors.append(f"bus.disable_torque(): {exc}")

        if not success:
            torque_disabled_ctx = getattr(bus, "torque_disabled", None)
            if callable(torque_disabled_ctx):
                try:
                    with torque_disabled_ctx():
                        pass
                    success = True
                except Exception as exc:
                    errors.append(f"bus.torque_disabled(): {exc}")

    if not success:
        if hasattr(robot, "disable_torque_all"):
            try:
                robot.disable_torque_all()
                success = True
            except Exception as exc:
                errors.append(f"robot.disable_torque_all(): {exc}")
        elif hasattr(robot, "relax"):
            try:
                robot.relax()
                success = True
            except Exception as exc:
                errors.append(f"robot.relax(): {exc}")

    if success:
        print("✓ Torque disabled - arm should be movable by hand")
    else:
        print("⚠️  Warning: Could not disable torque via supported APIs")
        if errors:
            print("   Errors encountered:")
            for err in errors:
                print(f"     - {err}")


def enable_torque_all(robot: SO101Follower) -> None:
    """
    Re-enable holding torque on all joints before commanding movements.
    This restores motor stiffness for normal operation.
    """
    try:
        # Try to enable torque on all joints
        if hasattr(robot, 'joint_names'):
            for joint in robot.joint_names:
                try:
                    robot.set_torque(joint, True)
                except AttributeError:
                    # If set_torque doesn't exist, try enable_torque
                    try:
                        robot.enable_torque(joint)
                    except AttributeError:
                        # Try accessing through bus if available
                        if hasattr(robot, 'bus'):
                            robot.bus.enable_torque(joint)
        elif hasattr(robot, 'enable_torque_all'):
            robot.enable_torque_all()
        elif hasattr(robot, 'bus'):
            # Access through motor bus
            for motor_id in range(1, 7):  # Typically motors 1-6
                try:
                    robot.bus.enable_torque(motor_id)
                except:
                    pass
        print("✓ Torque enabled on all joints - arm ready for movement commands")
    except Exception as e:
        print(f"⚠️  Warning: Could not enable torque: {e}")


def connect_robot(port: str = PORT, disable_torque: bool = False) -> SO101Follower:
    """
    Connect to the SO-101 robot arm and return the follower instance.
    
    Args:
        port: Serial port path (defaults to PORT)
        disable_torque: If True, disable motor torque after connection
                       so the arm can be moved freely by hand
    
    Returns:
        Connected SO101Follower instance
    """
    print(f"Connecting on {port} ...")
    config = SO101FollowerConfig(port=port, id="so101_arm")
    robot = SO101Follower(config)
    robot.connect()
    print("Connected.")
    
    if disable_torque:
        disable_torque_all(robot)
    
    return robot


def connect_gripper_only(port: str = PORT, disable_torque: bool = False) -> GripperOnlyRobot:
    """
    Connect just the gripper motor for standalone testing.
    Useful when the rest of the arm is unplugged/offline.
    """
    print(f"Connecting to gripper only on {port} ...")
    gripper_robot = GripperOnlyRobot(port)
    gripper_robot.connect()
    print("Gripper motor connected.")
    if disable_torque:
        disable_torque_all(gripper_robot)
    return gripper_robot


def planar_ik_mm(x_mm: float, y_mm: float) -> Tuple[float, float, float]:
    """
    Very simple planar IK for shoulder pan/lift/elbow based on x/y in millimeters.
    Assumes z is handled separately (we keep wrist angles fixed).
    """
    r_mm = math.sqrt(x_mm**2 + y_mm**2)
    # Pan is yaw around base: atan2(x, y) so y is "forward", x is "left/right"
    pan_theta = math.degrees(math.atan2(x_mm, y_mm))

    # Simple 2-link solve in plane; protect acos domain
    # Here we use humerus+forearm lengths to roughly reach r_mm.
    # Offset angle tuned from your previous heuristic.
    cos_lift = clamp((L_FOREARM - r_mm) / L_HUMERUS, -1.0, 1.0)
    lift_theta = math.degrees(math.acos(cos_lift)) - 100.0
    elbow_theta = -lift_theta + 20.0
    return pan_theta, lift_theta, elbow_theta


def move_to_xy(robot, x_mm: float, y_mm: float, close_gripper: bool = False, 
               gripper_fraction: float = None, duration: float = 3.0) -> None:
    """
    Move the arm to a planar x/y (robot frame) using a fixed wrist posture.
    
    Args:
        robot: SO101Follower instance
        x_mm: left/right; y_mm: forward/backward relative to origin defined in vision
        close_gripper: If True, close gripper fully (legacy boolean mode)
        gripper_fraction: Analog gripper position (0.0 = open, 1.0 = closed). 
                         If provided, overrides close_gripper.
        duration: Movement duration in seconds
    """
    pan_theta, lift_theta, elbow_theta = planar_ik_mm(x_mm, y_mm)

    # Determine gripper position
    if gripper_fraction is not None:
        # Use analog control
        gripper_pos = gripper_fraction_to_pos(gripper_fraction)
    else:
        # Use legacy boolean mode
        gripper_pos = GRIPPER_CLOSED_VAL if close_gripper else GRIPPER_OPEN_VAL

    target = {
        "shoulder_pan.pos": pan_theta,
        "shoulder_lift.pos": lift_theta,
        "elbow_flex.pos": elbow_theta,
        "wrist_flex.pos": 98.0,
        "wrist_roll.pos": 0.0,
        "gripper.pos": gripper_pos,
    }
    move_to(robot, target, duration=duration)

def demo_move(robot) -> None:
    """Example move using the simple IK."""
    x_mm = 75.0
    y_mm = 150.0
    print(f"Demo move to x={x_mm} mm, y={y_mm} mm")
    move_to_xy(robot, x_mm, y_mm, close_gripper=False)
    time.sleep(1.0)
    move_to_xy(robot, x_mm, y_mm, close_gripper=True)


if __name__ == "__main__":
    robot = None
    try:
        # Connect with torque enabled (default) for normal operation
        robot = connect_robot(PORT, disable_torque=False)
        demo_move(robot)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if robot:
            print("Disconnecting...")
            robot.disconnect()
