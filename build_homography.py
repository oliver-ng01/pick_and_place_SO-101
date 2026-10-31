"""
Build the camera -> table homography from the touched marks.

Reads table_points.csv (joint angles from measure_points.py, plus the pixel you clicked for
each mark), applies the joint correction, works out where the arm says each mark is, and
fits a homography from pixels to table positions in the robot's base frame (metres).
Saves it to homography.json for the hover test and the pipeline.

The data in table_points.csv was recorded with the OLD calibration (the one on the motors
since the 16 Oct revert). If the arm is recalibrated, re-touch the marks and refit the
joint correction before rebuilding this.

Run in the activated lerobot env:  python build_homography.py
"""

import csv
import json
import re
from datetime import date
from pathlib import Path

import cv2
import numpy as np
from lerobot.model.kinematics import RobotKinematics

URDF = Path(r"C:\Users\Oliver\Random Scripts\Robotics Control Project\SO101\so101_new_calib.urdf")
POINTS_CSV = Path("table_points.csv")
OUT = Path("homography.json")
IMAGE_SIZE = (640, 480)  # resolution the marks were clicked at (live cv2.VideoCapture frame)

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# Added to the arm's readings before FK; subtract from IK's answer before sending to the motors.
# Fitted 16 Oct 2026 from the ruler measurements (old calibration).
JOINT_CORRECTION_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": -4.3, "wrist_flex": -5.2, "wrist_roll": 0.0}


def corrected(joint_deg: dict[str, float]) -> np.ndarray:
    return np.array([joint_deg[n] + JOINT_CORRECTION_DEG[n] for n in ARM_JOINTS])


def pixel_to_table(H: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    """Pixels (N x 2) -> table positions (N x 2, metres, robot base frame)."""
    return cv2.perspectiveTransform(np.asarray(pixels, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)


def main() -> None:
    kinematics = RobotKinematics(str(URDF), "gripper_frame_link", ARM_JOINTS)

    pixels, table, heights = [], [], []
    with open(POINTS_CSV, newline="") as f:
        for row in csv.DictReader(f):
            joint_deg = {n: float(row[f"{n}_deg"]) for n in ARM_JOINTS}
            tip = kinematics.forward_kinematics(corrected(joint_deg))[:3, 3]
            u, v = map(float, re.findall(r"-?\d+", row["Coordinate of image"]))
            pixels.append([u, v]); table.append(tip[:2]); heights.append(tip[2])
    pixels, table, heights = np.array(pixels), np.array(table), np.array(heights)

    H, _ = cv2.findHomography(pixels, table)
    if H is None:
        raise SystemExit("findHomography failed: check the pixel and joint columns in the CSV")

    # How far the homography's answer is from where the arm says each mark is.
    fit_gap = np.linalg.norm(pixel_to_table(H, pixels) - table, axis=1) * 1000
    # Same, but each mark is left out of the fit it is tested against (the honest number).
    loo_gap = []
    for i in range(len(pixels)):
        keep = np.arange(len(pixels)) != i
        H_i, _ = cv2.findHomography(pixels[keep], table[keep])
        loo_gap.append(np.linalg.norm(pixel_to_table(H_i, pixels[i:i + 1])[0] - table[i]) * 1000)

    print("\nmark | pixel      | table x, y (cm)  | fit gap | left-out gap")
    for i, (p, t) in enumerate(zip(pixels, table)):
        print(f"  {i + 1}  | {p[0]:4.0f}, {p[1]:4.0f} | {t[0]*100:6.1f}, {t[1]*100:6.1f} | {fit_gap[i]:5.1f} mm | {loo_gap[i]:5.1f} mm")
    print(f"\nTypical gap: fit {np.sqrt(np.mean(fit_gap**2)):.1f} mm, left-out {np.sqrt(np.mean(np.square(loo_gap))):.1f} mm")
    print(f"Table height (mean touch z): {heights.mean()*100:.2f} cm, spread {np.ptp(heights)*1000:.1f} mm")

    OUT.write_text(json.dumps({
        "H_pixel_to_table_m": H.tolist(),
        "image_size": IMAGE_SIZE,
        "table_z_m": float(heights.mean()),
        "joint_correction_deg": JOINT_CORRECTION_DEG,
        "source": str(POINTS_CSV),
        "built": date.today().isoformat(),
    }, indent=2))
    print(f"Saved {OUT}")


if __name__ == "__main__":
    main()
