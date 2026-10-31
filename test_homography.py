"""
Hover test with live video: click a spot in the camera window, and the arm moves to hover the
gripper above it, pointing as close to straight down as it can reach. Click another spot at any
time and it heads there instead. Press q in the window to stop.

Run in the activated lerobot env:  python test_homography.py
"""

import os

# NumPy's maths library (MKL) and placo each load their own OpenMP runtime, and two in one
# process crash it. Single-threaded MKL never loads one. Must be set before numpy is imported.
os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")

import json

import cv2
import numpy as np
from lerobot.model.kinematics import RobotKinematics
from scipy.optimize import least_squares

from move_to_joints import ARM_JOINTS, connect_robot, read_joints, send_joints

URDF = r"C:\Users\Oliver\Random Scripts\Robotics Control Project\SO101\so101_new_calib.urdf"
HOMOGRAPHY_PATH = "homography.json"
HOVER_HEIGHT = 0.08  # metres above the table
MOVE_SECONDS = 3.0  # how long each move takes
PAN_AXIS_X = 0.0388  # shoulder_pan axis, metres in front of the base origin (from the URDF)
LOWER_DEG = np.degrees([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])  # URDF joint limits
UPPER_DEG = np.degrees([1.91986, 1.74533, 1.69, 1.65806, 2.84121])


def load_homography():
    with open(HOMOGRAPHY_PATH) as f:
        data = json.load(f)
    return np.array(data["H_pixel_to_table_m"]), data["table_z_m"], tuple(data["image_size"])


def pixel_to_table(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[:2] / p[2]  # divide by the third component (perspective divide)


def solve_ik(kin, q_now, xyz):
    """Joint angles (deg) that put the gripper tip at xyz, pointing as close to straight down as reachable.

    Wrist roll stays at its current value, so the gripper keeps its previous twist relative to the arm.
    Position is weighted far above tilt: the tip goes exactly to xyz, and the gripper tilts only
    where the wrist can't bend far enough to point straight down.
    """
    roll = q_now[4]

    def residuals(q4):
        T = kin.forward_kinematics(np.r_[q4, roll])
        return np.r_[(T[:3, 3] - xyz) * 1000, (T[:3, 2] - [0.0, 0.0, -1.0]) * 10]  # mm, then pointing-down error

    seed = q_now[:4].copy()
    seed[0] = -np.degrees(np.arctan2(xyz[1], xyz[0] - PAN_AXIS_X))  # start with the base already facing the target
    seed = np.clip(seed, LOWER_DEG[:4] + 1e-3, UPPER_DEG[:4] - 1e-3)
    sol = least_squares(residuals, seed, bounds=(LOWER_DEG[:4], UPPER_DEG[:4]))
    return np.r_[sol.x, roll]


def main():
    H, table_z, image_size = load_homography()
    kin = RobotKinematics(URDF, "gripper_frame_link", ARM_JOINTS)

    cap = cv2.VideoCapture(0)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Could not read from the camera")
    if (frame.shape[1], frame.shape[0]) != image_size:
        raise SystemExit(f"Camera gives {frame.shape[1]}x{frame.shape[0]}, homography was built at {image_size}")

    clicks = []  # filled by the mouse callback, emptied by the loop

    def on_click(event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))

    cv2.namedWindow("hover test", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("hover test", on_click)

    robot = connect_robot()
    try:
        q_current = read_joints(robot)  # current joint angles (corrected); during a move the loop updates it to each step's command
        steps = max(1, int(MOVE_SECONDS * 30))
        start, q_target, step = q_current, q_current, steps  # not moving yet
        boost = np.zeros(5)  # extra angle added to the command to make up for the motors sagging under gravity
        settle = 0  # frames since the last move finished
        dot, status = None, "Click a spot. r = report, q = quit."
        print(status)

        while True:
            ok, frame = cap.read()  # waits for the next camera frame, so the loop runs at the camera's rate
            if not ok:
                print("Lost the camera")
                break

            if step < steps:
                clicks.clear()  # still moving: ignore clicks until the move has finished
            elif clicks:
                u, v = clicks[-1]
                clicks.clear()
                x, y = pixel_to_table(H, u, v)
                xyz = np.array([x, y, table_z + HOVER_HEIGHT])
                q_new = solve_ik(kin, q_current, xyz)
                achieved = kin.forward_kinematics(q_new)
                error_mm = np.linalg.norm(achieved[:3, 3] - xyz) * 1000
                tilt_deg = np.degrees(np.arccos(np.clip(-achieved[2, 2], -1, 1)))
                if error_mm > 2:
                    status = f"Can't reach ({x*100:.1f}, {y*100:.1f}) cm: IK error {error_mm:.0f} mm. Not moving."
                else:
                    dot, start, q_target, step, settle = (u, v), q_current, q_new, 0, 0  # start a new move
                    status = f"Target ({x*100:.1f}, {y*100:.1f}) cm, tilt {tilt_deg:.1f} deg"
                print(status)

            if step < steps:
                step += 1
                q_current = start + (q_target - start) * step / steps
            else:
                settle += 1
                if settle % 10 == 0 and settle <= 60:  # for 2 s after a move, every third of a second: nudge by half the sag
                    gap = q_target - read_joints(robot)
                    gap[np.abs(gap) < 0.5] = 0  # ignore tiny gaps: chasing them makes the motors twitch back and forth
                    boost = np.clip(boost + 0.5 * gap, -10, 10)
            send_joints(robot, q_current + boost)  # keeps sending once the move is done, so it holds there

            if dot is not None:
                cv2.circle(frame, dot, 5, (0, 0, 255), -1)
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.imshow("hover test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):  # report: where the arm is told to be vs where it actually is
                q_actual = read_joints(robot)
                tip_told = kin.forward_kinematics(q_current)[:3, 3]
                tip_actual = kin.forward_kinematics(q_actual)[:3, 3]
                print("joints told   (deg):", np.round(q_current, 1))
                print("joints actual (deg):", np.round(q_actual, 1))
                print(f"tip told   x={tip_told[0]*100:.1f} y={tip_told[1]*100:.1f} z={tip_told[2]*100:.1f} cm")
                print(f"tip actual x={tip_actual[0]*100:.1f} y={tip_actual[1]*100:.1f} z={tip_actual[2]*100:.1f} cm")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        input("Hold the arm, then press Enter to let go (it goes limp)")
        robot.disconnect()


if __name__ == "__main__":
    main()
