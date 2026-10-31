import os
os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")
import cv2 as cv
import numpy as np
from box_footprint import box_center, to_pixels, to_table
from masking import color_dict_HSV
from test_homography import load_homography,solve_ik
from lerobot.model.kinematics import RobotKinematics
from move_to_joints import connect_robot,move_to_joints,read_joints,open_gripper,close_gripper
URDF = r"C:\Users\Oliver\Random Scripts\Robotics Control Project\SO101\so101_new_calib.urdf"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
HOVER_HEIGHT = 0.07
MOVE_SECONDS =3.0
GRASP_HEIGHT = 0.015  # metres above the table for the gripper point; tune for your box
BLUE_LOW = np.array(color_dict_HSV['blue'][1])
BLUE_HIGH = np.array(color_dict_HSV['blue'][0])
# Rest pose: read from the arm on 17 Oct 2026 with read_joints (corrected angles, degrees).
REST_POSE = np.array([88.4, -113.4, 92.0, 64.9, 0.8])


def find_blue(frame):
    """Pixel centre of the biggest blue blob, or None."""
    mask = cv.inRange(cv.cvtColor(frame, cv.COLOR_BGR2HSV), BLUE_LOW, BLUE_HIGH)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    M = cv.moments(max(contours, key=cv.contourArea))
    if M['m00'] == 0:
        return None
    return np.array([M['m10'] / M['m00'], M['m01'] / M['m00']])


def live_preview(cap, H):
    """Show the camera with both boxes marked until Enter/g (returns (green, blue)) or q (returns None)."""
    while True:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit("Could not read from the camera")

        # detect both boxes on the clean image first, then draw
        green = box_center(frame, H)  # (centre, corners, sides_used) in table metres, or None
        blue_px = find_blue(frame)
        blue = to_table(H, blue_px[None])[0] if blue_px is not None else None  # pixels -> table metres

        if green is not None:
            centre, corners, sides_used = green
            cv.polylines(frame, [to_pixels(H, corners).astype(np.int32)], True, (0, 0, 255), 2)  # footprint, red
            cv.circle(frame, tuple(to_pixels(H, centre[None])[0].astype(int)), 4, (255, 0, 0), -1)  # centre, blue
            green_text = f"green box: x={centre[0]*100:.1f} y={centre[1]*100:.1f} cm"
        else:
            green_text = "green box: not found"
        if blue is not None:
            cv.circle(frame, tuple(blue_px.astype(int)), 4, (255, 255, 0), -1)  # blue box centre, cyan
            blue_text = f"blue box: x={blue[0]*100:.1f} y={blue[1]*100:.1f} cm"
        else:
            blue_text = "blue box: not found"
        cv.putText(frame, green_text, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(frame, blue_text, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(frame, "Enter or g = go, q = quit", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.imshow('frame', frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            return None
        if key in (13, ord('g')):  # 13 = Enter
            if green is None or blue is None:
                print("Need both boxes in view before going")
            else:
                return green, blue


H, table_z, image_size = load_homography()
kin = RobotKinematics(URDF, "gripper_frame_link", ARM_JOINTS)
robot = connect_robot()  # from here on the motors hold the arm wherever it is
cap = cv.VideoCapture(0)

try:
    while True:
        print("Live preview: arrange the boxes, then Enter or g = go, q = quit")
        boxes = live_preview(cap, H)
        if boxes is None:
            break
        (centre, corners, sides_used), blue = boxes
        x, y = centre
        bbx, bby = blue
        print(f"Green box centre: x={x*100:.1f} cm, y={y*100:.1f} cm (from {sides_used} bottom edge(s))")
        print(f"Blue box: x={bbx*100:.1f} cm, y={bby*100:.1f} cm")

        xyz = np.array([x, y, table_z + HOVER_HEIGHT])

        q_current = read_joints(robot)
        q_target = solve_ik(kin, q_current, xyz)

        move_to_joints(robot,q_target,seconds=MOVE_SECONDS,fps=30)
        scale = open_gripper(robot)


        xyz_down = np.array([x, y, table_z + GRASP_HEIGHT])
        q_down = solve_ik(kin, q_target, xyz_down)   # start the IK from the hover pose
        move_to_joints(robot, q_down, seconds=2.0, fps=30)

        close_gripper(robot,scale)

        move_to_joints(robot,q_target,seconds=MOVE_SECONDS,fps=30)
        #move it back up to q_target

        xyz_end = np.array([bbx, bby, table_z + HOVER_HEIGHT])
        q_end = solve_ik(kin, q_target, xyz_end)   # start the IK from the hover pose
        move_to_joints(robot, q_end, seconds=2.0, fps=30)

        scale = open_gripper(robot)

        # back to the rest pose, gripper closed; the motors hold it there while you rearrange
        close_gripper(robot, scale)
        move_to_joints(robot, REST_POSE, seconds=MOVE_SECONDS, fps=30)
        print("Done. Rearrange the setup for the next run.")
finally:
    cap.release()
    cv.destroyAllWindows()
    input("Finished. Hold the arm, then press Enter to let go (it goes limp)")
    robot.disconnect()
