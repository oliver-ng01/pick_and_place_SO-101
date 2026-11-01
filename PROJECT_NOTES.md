# Classical pick-and-place on the SO-101: project summary and arm notes

## 1. What this project is

A **classical** (no learning) pick-and-place on the SO-101 follower arm: a webcam finds a green box
on the table, the arm picks it up and drops it in a blue box, then returns to a rest pose. Built as
the hand-coded baseline for the SmolVLA vs ACT vs classical comparison.

**Results** (full breakdown in [classical_imitation_comparison.md](classical_imitation_comparison.md)):

- **Classical (17 Oct 2026):** 25 tries, 23 successes (92%), 2 fails. 2 of the successes were
  "dubious" (picked the box up across its whole width instead of by one wall). One fail dragged the
  box along the table and pushed the blue box; the green box may have been outside the area the
  homography was calibrated for.
- **ACT (31 Oct 2026):** 25 tries, 12 successes (48%). 12 / 19 with the green box to the right of the
  blue box, 0 / 6 with it on the left, where it tends to head for the blue box's side instead.

### Pipeline

```
camera frame (640×480)
  → box_footprint.box_center      green box: bottom edges → table position of its centre
  → find_blue (main.py)           blue box: blob centre → table position
  → test_homography.solve_ik      joint angles to hover 7 cm above / descend to 1.5 cm over the box
  → move_to_joints.move_to_joints smooth move, then 2 s of sag correction
  → open / close gripper, lift, move over the blue box, release
  → back to REST_POSE; repeat (Enter / g in the preview window)
```

### Files

| File | What it does |
|---|---|
| `main.py` | The pick-and-place loop with live preview. Run this. |
| `box_footprint.py` | Green-box centre from its bottom edges. |
| `box_footprint explained.md` | `box_footprint.py` line by line. Its `main.py` section describes the 17 Oct one-shot hover version, not the current loop. |
| `build_homography.py` | Builds `homography.json` (pixels → table, robot base frame, metres) from `table_points.csv`. |
| `homography.json` | The fitted homography, table height, image size and joint correction (built 16 Oct 2026). |
| `table_points.csv` | The 8 touched marks: arm joint angles, FK position, clicked pixel and ruler measurement. |
| `measure_points.py` | Records table points by touching marks with the arm (forward kinematics). Writes `table_points2.csv` by default. |
| `table_points2.csv` | Scratch output of `measure_points.py` (one point); not used by the build. |
| `test_homography.py` | Click-to-hover test; also holds `solve_ik` and `load_homography`. |
| `move_to_joints.py` | Connect, read and command the arm; joint correction; sag correction; gripper. |
| `masking.py` | Colour-mask experiments and the HSV colour table. |
| `get_marker_pixel_pos.py` | Click on a camera frame to print pixel coordinates (used to locate the marks). |
| `video.py` | Minimal webcam test (greyscale preview). |
| `media/` | Setup photo, test shots of the box on marks 3 and 6, and GIFs of both approaches running. |
| `classical_imitation_comparison.md` | Classical vs ACT results. |
| `README.md` | Overview, results, how to run, and how to put the robot and camera back in the same place. |

### Measured accuracy

| What | How good | How measured |
|---|---|---|
| Camera → table (homography) | 3–6 mm inside the marked area; ~8 mm overall, worse at the edges | each of the 8 marks predicted from the other 7 |
| Green-box centre | 4 mm (box turned), 6 mm (box straight on) | box centred on marks 3 and 6 |
| IK | < 0.2 mm (in the model) | all marks, several start poses |
| Arm reaching its command | within 0.5° per joint after sag correction | `r` report on the real arm |

---

## 2. Arm facts other projects need to know

### Use the calibration ID `orange_follower_arm`, always

- The follower's calibration lives in WSL:
  `~/.cache/huggingface/lerobot/calibration/robots/so_follower/orange_follower_arm.json`,
  with an identical copy in `C:\Users\Oliver\.cache\huggingface\lerobot\calibration\robots\so_follower\`.
  Everything in this project (joint correction, homography, rest pose) depends on **this** calibration
  being on the motors.
- **If LeRobot asks "Press ENTER to use provided calibration file … or type 'c'":**
  - with `orange_follower_arm` → Enter is safe (it writes the right calibration back onto the motors);
  - with any other ID (e.g. the old `my_awesome_follower_arm`) → **Ctrl+C**, and rerun with
    `--robot.id=orange_follower_arm`. Pressing Enter there overwrites the motors with a different
    calibration (this happened around 20 Oct 2026).
  - Never type `c` unless you mean to recalibrate (see below).
- **Recalibrating** changes every joint reading. Afterwards the joint correction, `table_points.csv`,
  `homography.json`'s table height and `REST_POSE` are all invalid until re-measured. A careful
  recalibration on 16 Oct 2026 came out slightly *worse*, so it was reverted.
- LeRobot's degree mode puts 0° at the **middle of the range swept during calibration**, not at the
  pose you hold (except wrist roll). Stopping short of a hard stop during the sweep shifts that
  joint's zero by half the shortfall.

### Joint correction: elbow −4.3°, wrist flex −5.2°

- **What:** this arm's elbow and wrist-flex readings are a few degrees off from what the URDF
  (`so101_new_calib.urdf`) assumes. Found on 16 Oct 2026 by touching 8 table marks with the arm and
  comparing with ruler measurements; it cut the camera-vs-arm mismatch from 11.3 mm to 3.4 mm.
- **How to use it:** add it to the arm's readings before forward kinematics or IK; subtract it from
  IK's answer before commanding the motors (`read_joints` / `send_joints` in `move_to_joints.py`
  do both).
- **Transfers to:** any project on **this arm, with the `orange_follower_arm` calibration, using this
  URDF** (frame `gripper_frame_link`) for FK/IK or for replaying real joint readings in simulation.
  Most accurate with the gripper pointing down near table height; the data couldn't fully separate
  "elbow + wrist flex" from "shoulder lift + elbow", so very different poses can be a few mm worse.
- **Does not transfer to:** another arm (e.g. the Hardware Lab chess rig), this arm after a
  recalibration, or **learned policies (ACT, SmolVLA)**. Those work in the arm's raw joint readings
  and never use the URDF: don't apply the correction to their data or actions.

### Gravity sag and the fix (transfers to anything)

- LeRobot sets the servos to half stiffness (`P_Coefficient` 16) with no integral term, so under load
  the arm stops short of its target: ~3.5–4° at the elbow, ~2° at the shoulder lift when stretched
  out, enough to put the gripper 1–2 cm low.
- `move_to_joints` fixes it after each move by nudging the command by half the remaining error every
  third of a second for 2 s (an integral correction), ignoring errors under 0.5° and capped at ±10°.
  It needs no model, so it works on any arm and any project.

### Other arm / URDF facts

- In the URDF base frame: the shoulder-pan axis is **3.88 cm in front of** the base origin, and a
  **positive pan** swings the tip towards **−y**.
- `gripper_frame_link` (LeRobot's tool point) sits ~8 mm off the wrist-roll axis, so changing wrist
  roll moves it slightly. Keep the roll consistent between measuring and grasping.
- Pointing straight down 5–8 cm above the table only works up to ~25 cm from the pan axis; further
  out the wrist-flex limit (95° in the URDF) forces up to ~16° of tilt.
- `REST_POSE` (shoulder lift −113.4°) is beyond the URDF's ±100° limit: fine for direct joint moves,
  but IK will never return it.
- LeRobot's own `RobotKinematics.inverse_kinematics` does one solver step per call and only takes a
  full 6-D target; on this 5-joint arm it got stuck or swung the wrist roll. This project's
  `solve_ik` uses SciPy `least_squares` with LeRobot's forward kinematics and the URDF joint limits.

### Software environment (Windows `lerobot` conda env, `D:\anaconda3\envs\lerobot`)

- **Activate it first** (Anaconda Prompt → `conda activate lerobot`); running its `python.exe`
  directly breaks NumPy's maths library.
- Put `os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")` at the very top of any script, before
  NumPy is imported, or NumPy and placo load two OpenMP libraries and the process aborts
  ("OMP: Error #15").
- `placo` 0.10.0 is installed from conda-forge (LeRobot asks for 0.9.x, which has no Windows build);
  Pinocchio was upgraded to 4.1 with it.
- The normal `opencv-python` replaced `opencv-python-headless` so windows work. `pip check` warns that
  LeRobot wants the headless one; that's expected. If a LeRobot reinstall brings headless back, both
  will clash: uninstall both, reinstall `opencv-python`.

---

## 3. Setup-specific: not reusable elsewhere

- `homography.json` and `table_points.csv`: only valid for **this camera position, this table and
  640×480 frames**. Move the camera and you need to re-click the 8 marks and rebuild.
- Colour ranges: green brightness minimum is **30** (not 70) so the box still counts in the robot's
  shadow. The blue detection can lock onto the **robot's own blue part**, so check the cyan dot is on
  the blue box before pressing go.
- Box size in `box_footprint.py` (5.15 × 3.5 cm footprint, 2.7 cm tall) is for this green box only.
- Hard-coded paths and ports: the arm on `COM5` and the WSL calibration folder (`CALIB_DIR`) in
  `move_to_joints.py`; the URDF path (`..\Robotics Control Project\SO101\so101_new_calib.urdf`, not
  in this repo) in `main.py`, `test_homography.py`, `build_homography.py` and `measure_points.py`.
  Change these on another PC.
