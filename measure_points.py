"""
Measure points on the table with the SO-101 follower arm, using forward kinematics.

Move the gripper tip onto a mark by hand, press Enter, and the script prints where the
tip is in the URDF's base frame (the same frame IK will use later). Points are saved to
a CSV for the pixel -> table homography.

Forward kinematics is LeRobot's own `RobotKinematics` (lerobot/model/kinematics.py), which
also has `inverse_kinematics` for the grasping step later. It needs `placo` (installed
from conda-forge, 0.10.0; LeRobot asks for 0.9.x, which has no Windows build).

Runs in the Windows `lerobot` conda env (LeRobot 0.5.2). Activate it first (Anaconda
Prompt, then `conda activate lerobot`): running its python.exe directly without
activating makes NumPy maths crash.

Usage:
    python measure_points.py --test
        No robot needed. Checks LeRobot's kinematics against Pinocchio.
    python measure_points.py --port COM5 --robot-id orange_follower_arm
        Live measuring. Find the port with `lerobot-find-port`.

placo prints a "self collisions in neutral position" warning when it loads the URDF.
That is about neighbouring links' meshes touching and doesn't affect positions.

Safety: this script never calibrates the arm. It only reads the existing calibration
file, so your recorded demos stay consistent. It switches motor torque OFF so you can
move the arm by hand: hold the arm when it does, or it will drop.
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from lerobot.model.kinematics import RobotKinematics

DEFAULT_URDF = Path(
    r"C:\Users\Oliver\Random Scripts\Robotics Control Project\SO101\so101_new_calib.urdf"
)
# The arm was calibrated under WSL, so its calibration files live there.
DEFAULT_CALIB_DIR = Path(
    r"\\wsl.localhost\Ubuntu-24.04-D\home\oliver\.cache\huggingface\lerobot\calibration\robots\so_follower"
)
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
TIP_FRAME = "gripper_frame_link"  # LeRobot's default end-effector frame


def tip_position(kinematics: RobotKinematics, joint_deg: dict[str, float]) -> np.ndarray:
    """Tip position (metres, URDF base frame) from LeRobot joint readings in degrees."""
    angles = np.array([joint_deg[name] for name in ARM_JOINTS])
    return kinematics.forward_kinematics(angles)[:3, 3]


def run_test(kinematics: RobotKinematics, urdf_path: Path) -> None:
    """Compare LeRobot's kinematics with Pinocchio's on random joint angles."""
    zero = tip_position(kinematics, {name: 0.0 for name in ARM_JOINTS})
    print(f"Tip at all-zero joints: x={zero[0]*100:.1f} y={zero[1]*100:.1f} z={zero[2]*100:.1f} cm")

    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    frame_id = model.getFrameId(TIP_FRAME)

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        angles = rng.uniform(-90, 90, size=len(ARM_JOINTS))
        q = pin.neutral(model)
        for name, angle in zip(ARM_JOINTS, angles):
            q[model.joints[model.getJointId(name)].idx_q] = np.deg2rad(angle)
        pin.framesForwardKinematics(model, data, q)
        ours = tip_position(kinematics, dict(zip(ARM_JOINTS, angles)))
        worst = max(worst, np.linalg.norm(ours - data.oMf[frame_id].translation))
    print(f"Largest difference from Pinocchio over 200 poses: {worst*1000:.4f} mm")


def run_live(kinematics: RobotKinematics, args: argparse.Namespace) -> None:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    config = SO101FollowerConfig(port=args.port, id=args.robot_id, calibration_dir=args.calib_dir)
    robot = SO101Follower(config)
    if not robot.calibration:
        sys.exit(f"No calibration file at {robot.calibration_fpath}. Stopping rather than recalibrating.")

    robot.connect(calibrate=False)
    rows = []
    try:
        input("Hold the arm now. Press Enter to switch motor torque OFF...")
        robot.bus.disable_torque()
        print("Torque off. Put the gripper tip on a mark, then press Enter. Type q and Enter to finish.")

        while True:
            label = input(f"\nPoint {len(rows) + 1} (Enter to record, q to quit): ").strip()
            if label.lower() == "q":
                break
            joints = robot.bus.sync_read("Present_Position")
            tip = tip_position(kinematics, joints)
            print("  joints (deg): " + ", ".join(f"{n}={joints[n]:.1f}" for n in ARM_JOINTS))
            print(f"  tip: x={tip[0]*100:.1f} y={tip[1]*100:.1f} z={tip[2]*100:.1f} cm")
            rows.append([len(rows) + 1, *tip, *(joints[n] for n in ARM_JOINTS)])
    finally:
        robot.disconnect()

    if rows:
        with open(args.out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["point", "x_m", "y_m", "z_m", *(f"{n}_deg" for n in ARM_JOINTS)])
            writer.writerows(rows)
        print(f"\nSaved {len(rows)} points to {args.out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--test", action="store_true", help="check the kinematics without a robot")
    parser.add_argument("--port", help="serial port of the follower arm, e.g. COM5")
    parser.add_argument("--robot-id", default="orange_follower_arm", help="calibration file name, without .json")
    parser.add_argument("--calib-dir", type=Path, default=DEFAULT_CALIB_DIR)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--out", type=Path, default=Path("table_points2.csv"))
    args = parser.parse_args()

    kinematics = RobotKinematics(str(args.urdf), TIP_FRAME, ARM_JOINTS)
    if args.test:
        run_test(kinematics, args.urdf)
    elif args.port:
        run_live(kinematics, args)
    else:
        parser.error("give --test, or --port to measure with the robot")


if __name__ == "__main__":
    main()
