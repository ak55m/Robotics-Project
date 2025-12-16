# ArUco Workspace Grid & Robot Calibration Guide

This guide walks through the complete workflow for mapping the physical workspace into a named grid, streaming the camera feed with overlays, recording robot calibration poses per grid cell, and building a dictionary so the robot can pick up objects placed at known grid centers. Every step below is designed so you can go from a blank board to “click a cell → arm performs the correct motion with the correct gripper behavior.”

---

## 1. Physical Setup

![Workspace ArUco grid with slots](img/board_grid_named.jpg)

1. **Place four large ArUco markers** at the corners of the board (IDs 0–3 using the 4×4 dictionary or IDs 10–13 for the 5×5 set—just keep them consistent).
2. **Measure the rectangle** between the four corner tags. In our rig this is roughly 24.5" × 15" (≈ 622 mm × 381 mm).
3. **Draw the grid on the board** (optional but recommended) using `IMG_9051_grid_named.jpg` as the single reference image.  
   - The image shows the ArUco boundary (red), the internal grid (green, 4×4 by default), and the cell names (`R1C1` … `R4C4`).
   - For a 4×4 breakdown of a 24.5" × 15" area, each cell is ~6.125" wide × 3.75" tall (≈ 155.6 mm × 95.25 mm). Mark these dimensions on the board so the physical grid matches the camera overlay.  
   - Remember the robot’s reach limits: if the gripper is too low in any cell, raise the stored `shoulder_lift` or `wrist_flex` ticks by a few dozen counts in `grid_pose_map.json` so the pose stops before scraping the table.

---

## 2. Streaming the Camera Feed With Bounds + Grid

Use the ArUco detection pipeline to display the workspace live until the overlay perfectly matches the physical board:

```bash
python aruco_grid_visualizer.py --camera-index 0 --grid 4 4 --label-grid
```

- The script detects the four outer ArUco markers, draws the red bounding quadrilateral, and overlays the green grid with labels (`R1C1` etc.).
- When streaming, verify the overlay matches the physical grid drawn on the board. Adjust camera height/angle if the grid skews heavily.

Once the overlay looks correct, you can confirm the calibration and command moves directly from the live feed (IK mode):

```bash
python aruco_click_move.py --live --camera-index 0 --label-grid \
    --connect --save-calibration camera_cal.json
```

This computes the pixel→mm calibration from the camera stream, draws the same grid overlay, and lets you left-click any grid cell to send the robot to that location (assuming `--connect`). The calibration is saved for future use.

---

## 3. Naming & Recording Grid Cells

### 3.1 Grid Naming

- We use `R{row}C{column}` names. Example: `R1C1` is the top-left cell, `R4C4` is the bottom-right.
- When drawing the grid physically, mark each cell with the same label (painter’s tape + marker works well).

### 3.2 Recording Robot Poses For Each Cell

We'll capture a ground-truth pose and joint configuration for each cell to create a movement dictionary. `grid_pose_capture.py` is the canonical script for this now; it writes straight to `grid_pose_map.json` and guarantees the open/closed gripper ticks are recorded alongside the joints.

1. **Launch the capture tool (it talks directly to `grid_pose_map.json`):**

   ```bash
   python grid_pose_capture.py --pose-file grid_pose_map.json
   ```

2. For each entry you want to store:
   - Choose `g` for a grid cell or `d` for the default/off-grid pose.
   - Type the grid label (e.g., `R2C3`). The script disables torque so you can manually move the arm to the exact pickup/drop point.
   - Press Enter when satisfied; torque re-enables and the joint ticks are captured automatically.
   - **Gripper calibration happens immediately afterward.** The tool explicitly asks you to (a) set the gripper to an OPEN state and press Enter (tick stored), then (b) set the gripper to the CLOSED/HOLD state and press Enter (tick stored). No extra typing is required because the raw values come straight from the robot.
   - Repeat for as many grid cells (and for the default pose) as needed, then press `s` to save back to disk.

The resulting `grid_pose_map.json` contains:

```json
{
  "default_pose": {
    "label": "default",
    "joints": { ... },
    "gripper_open": 3315,
    "gripper_closed": 1872
  },
  "grids": {
    "R3C2": {
      "label": "R3C2",
      "joints": { ... },
      "gripper_open": 3315,
      "gripper_closed": 1872
    }
  }
}
```

> Tip: if a joint scrapes the table when replayed, edit the stored tick in `grid_pose_map.json` (or re-capture the pose) so the arm stops sooner. Shoulder lift and wrist flex are the key ones for vertical clearance.

---

## 4. Building the Movement Dictionary

Once the dataset is recorded, convert it into the canonical pose map (`grid_pose_map.json`). Each entry holds:

- `joints`: the raw tick values for `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, and `wrist_roll`.
- `gripper_open` / `gripper_closed`: the ticks captured when the gripper was open vs. holding an object in that pose.

### 4.1 `grid_pose_runner.py` – replay one or more poses from the JSON

```bash
python grid_pose_runner.py default R3C2 R3C3 --segments 10 --segment-duration 1.2
```

- Enter the gripper state you want before the queue (`open`/`close`/`hold`/`skip`) and after the queue finishes. The runner applies the stored ticks so you are not hard-coding gripper positions.
- The helper automatically interpolates between the current pose and each target pose in the queue. If the arm is already within ±5 ticks of a pose, it skips commanding that joint.
- After the final pose completes, the script waits ~5 s before disconnecting so you can verify the gripper action finished cleanly.
- If you queue multiple labels, the “before” gripper action happens once prior to the first pose and the “after” gripper action occurs once after the final pose—perfect for routines like “open → move to default → move to R3C2 → close”.

### 4.2 `aruco_click_move.py` – click a grid cell to run the saved pose

You can now click a live camera feed or static image and have the robot execute the corresponding grid pose (plus an optional default pose first). Example:

```bash
python aruco_click_move.py --live --camera-index 0 \
    --grid-pose-map grid_pose_map.json --default-grid-label default \
    --width-mm 622.3 --height-mm 381.0 --grid-rows 4 --grid-cols 4 \
    --pose-runner-segments 10 --pose-runner-segment-duration 1.0
```

- When `--grid-pose-map` is provided, the script forces the grid labels on screen and maps each click to an `R#C#` label using the pixel→mm calibration.
- Each click prompts for the gripper state **before** the queued move and **after** all queued poses finish. Valid values are `open`, `close`, `hold`, or `skip` (blank input also means skip). The commands reuse the exact ticks recorded during capture.
- The queue typically contains `[default, clicked_label]`, so the arm retreats to the lift/default pose before moving into the cell. Set `--default-grid-label ""` if you only want the clicked cell.
- `--connect` is ignored in this mode because the helper spins up its own connection inside the queue runner. Just make sure the robot is powered and reachable on `so101_control.PORT`.
- If the script can’t find a pose for the clicked label, it warns you without moving the robot, so you can capture that cell first.

This is the fastest way to confirm a new pose: stream the overlay, click a cell, choose how the gripper should behave, and watch the robot reproduce the saved motion.

---

## 5. Recommended Workflow

1. **Draw and label the grid** referencing `IMG_9051_grid_named.jpg`.
2. **Run the live detection script** (`aruco_grid_visualizer.py`) to verify the bounds overlay lines up with the physical grid.
3. **Capture robot poses** for each cell using `grid_pose_capture.py`, storing them in `grid_pose_map.json`.
4. **Use `aruco_click_move.py --live --grid-pose-map grid_pose_map.json`** to click any cell in the live feed and confirm the robot moves there using the saved poses (no raw coordinate guessing needed anymore).
5. **Integrate vision** so that when the system detects a knob at cell `R2C3`, it calls the corresponding pose and executes the pickup routine (either via `grid_pose_runner.py`, from `aruco_click_move.py`, or by importing `execute_pose_sequence` into your own control script).
6. **Queue multiple poses if needed** with `grid_pose_runner.py` (e.g., `python grid_pose_runner.py default R3C3`). Gripper actions are asked once before the queue and once after, and their tick values come from the captured data.

---

## Files in This Workflow

| File | Purpose |
| --- | --- |
| `aruco_grid_visualizer.py` | Shows the live camera feed with ArUco bounds and labeled grid to verify alignment before recording. |
| `grid_pose_capture.py` | Prompts you to move the robot to each grid cell (or the default pose), records joint angles, asks you to set the gripper OPEN then CLOSED so the ticks are stored, and writes everything into `grid_pose_map.json`. |
| `grid_pose_runner.py` | Replays any stored pose at raw tick level (single pose or a queue), applying your chosen gripper action before the queue and again after it completes. |
| `aruco_click_move.py` | In IK mode it still converts pixels → XYZ, but with `--grid-pose-map` it lets you click a labeled grid cell on the live feed and automatically runs the stored poses using the runner helper. |

### Robot Setup

1. Connect the SO-101 arm to your computer via the USB port defined in `so101_control.PORT`.
2. Ensure the robot is powered and that the Lerobot dependencies from `requirements.txt` are installed.
3. When running scripts that move the arm (`grid_pose_capture.py`, `aruco_click_move.py --connect`), keep the workspace clear and supervise the arm.

With these steps complete, placing an object at the center of any named grid square gives the robot a direct, pre-calibrated motion to pick it up, including the correct gripper behavior captured per pose.
