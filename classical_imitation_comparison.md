# Results: classical vs learned pick-and-place

The task in hand is to pick up a green box and place it on the blue box, using classical robotics and imitation learning techniques

## Summary

| | Classical (`main.py`) | ACT (learned) |
|---|---|---|
| Tries | 25 | 25 |
| Successes | **23 (92%)** | **12 (48%)** |
| Fails | 2 | 13 |
| Dubious successes (included in successes) | 2 | not recorded |

---

## Classical (hand-coded, run 17 Oct 2026)

| Tries | Successes | Fails | Dubious successes |
|---|---|---|---|
| 25 | 23 | 2 | 2 |

- **Dubious success:** the gripper picked the box up across its whole width instead of by one side.
  If these count as fails, the success rate is 21 / 25 (84%).


---

## ACT (learned policy)

An ACT policy trained on 50 teleoperated demos, run on the arm with a wrist camera and a tripod scene camera across different configurations of green and blue box placements

| Green box position | Tries | Successes | Fails | Success rate |
|---|---|---|---|---|
| Right of the blue box | 19 | 12 | 7 | 63% |
| Left of the blue box | 6 | 0 | 6 | 0% |
| **Total** | **25** | **12** | **13** | **48%** |

### Observations

- It does well when the green box is to the right of the blue box.
- With the green box on the left and the blue box on the right, it heads for the blue box's
  side and stalls. This might have come from the training data (for example, too few demos with that layout) causing the policy to fix to always going for the object to the right.
- When it picks the green box up across its whole width, it doesn't know to put it down, which is to be expected since none of the training set involves whole width picking.
- Sometimes, after missing on the first try, it corrects itself well, and when moving the box during inference it is able to adapt.

---

## Comparison

| Classical | Imitation Learning |
|---|---|
| High success rate compared to imitation learning | High adaptability to change in environment e.g. changing positions of box midway|
| No training needed| Need to record training episodes and lengthy fine-tuning (~4 hours)|
|More flexible setup, easier to recalibrate when setup changes| Rigid setup, if the setup changes need to record all 50 episodes again|
| Requires prior knowledge of the green box (its dimensions)| No prior knowledge needed|
|Needs to account for effects e.g. gravity sag and joint friction, as well as calibration offsets|Offsets are already automatically accounted for by the policy|
|Does not require wrist camera|Requires wrist camera|
|Can run at higher speeds|Speed limited by inference time|
|More general (succeed when green box is to the left)| Performance restricted by dataset|
