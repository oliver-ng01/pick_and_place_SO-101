import os

os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")  # avoid the OpenMP clash with placo (see test_homography.py)

import time
from pathlib import Path

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# Added to the arm's readings before FK/IK; subtracted from IK's answer before sending to the motors.
# Fitted 16 Oct 2026 from the ruler measurements (old calibration).
JOINT_CORRECTION_DEG = np.array([0.0, 0.0, -4.3, -5.2, 0.0])
CALIB_DIR = Path(r"\\wsl.localhost\Ubuntu-24.04-D\home\oliver\.cache\huggingface\lerobot\calibration\robots\so_follower")


def connect_robot(port: str = "COM5") -> SO101Follower:
    config = SO101FollowerConfig(
        port=port,
        id="orange_follower_arm",
        calibration_dir=CALIB_DIR,
        max_relative_target=10.0,  # safety net: never more than 10 degrees ahead of where the motor is
    )
    robot = SO101Follower(config)
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise SystemExit("Motors don't match the calibration file. Run lerobot-calibrate and press Enter first.")
    return robot


def read_joints(robot: SO101Follower) -> np.ndarray:
    """Current arm joint angles in degrees, with the correction applied (the angles FK/IK use)."""
    pos = robot.bus.sync_read("Present_Position")
    return np.array([pos[n] for n in ARM_JOINTS]) + JOINT_CORRECTION_DEG


def send_joints(robot: SO101Follower, q: np.ndarray) -> None:
    """Command q (degrees, corrected angles as used by FK/IK): the correction is removed for the motors."""
    motor = np.asarray(q) - JOINT_CORRECTION_DEG
    robot.send_action({f"{n}.pos": float(v) for n, v in zip(ARM_JOINTS, motor)})

def open_gripper(robot,scale=50,seconds: float = 2.0, fps: int = 30):
    steps = max(1, int(seconds * fps))
    
    for i in range(1, steps + fps + 1):  # the extra second holds the final target while it settles
        robot.send_action({"gripper.pos": scale*min(i, steps) / steps})
        time.sleep(1 / fps)
    return scale
def close_gripper(robot,scale,seconds: float = 2.0, fps: int = 30):
    """
    scale is the current gripper scaling, determined by scale in open_gripper
    """
    steps = max(1, int(seconds * fps))
    
    for i in range(1, steps + fps + 1):  # the extra second holds the final target while it settles
        robot.send_action({"gripper.pos": scale*(1- min(i, steps) / steps)} )
        time.sleep(1 / fps)

def move_to_joints(robot: SO101Follower, q: np.ndarray, seconds: float = 3.0, fps: int = 30) -> None:
    """Move smoothly to q (degrees, corrected angles as used by FK/IK).
    
         
    """
    start = read_joints(robot)
    steps = max(1, int(seconds * fps))
    for i in range(1, steps + fps + 1):  # the extra second holds the final target while it settles
        send_joints(robot, start + (np.asarray(q) - start) * min(i, steps) / steps)
        time.sleep(1 / fps)
    settle = 0
    boost = np.zeros(5)
    while settle <=60 :
        if settle % 10 == 0:  # for 2 s after a move, every third of a second: nudge by half the sag
            gap = q - read_joints(robot)
            gap[np.abs(gap) < 0.5] = 0  # ignore tiny gaps: chasing them makes the motors twitch back and forth
            boost = np.clip(boost + 0.5 * gap, -10, 10)
        send_joints(robot, np.asarray(q) + boost)  # actually command the boosted target
        time.sleep(1 / fps)  # one frame per round, so the 60 rounds take 2 s and the arm has time to respond
        settle += 1
