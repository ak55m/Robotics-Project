# ArUco Workspace Grid & Robot Calibration Guide

This guide walks through the complete workflow for mapping the physical workspace into a named grid, streaming the camera feed with overlays, recording robot calibration poses per grid cell, and building a dictionary so the robot can pick up objects placed at known grid centers.

---

## 1. Physical Setup

![Workspace ArUco grid with slots](img/board_grid_named.jpg)

1. **Place four large ArUco markers** at the corners of the board (IDs 0–3 using the 4×4 dictionary or IDs 10–13 for the 5×5 set—just keep them consistent).
2. **Measure the rectangle** between the four corner tags. In our rig this is roughly 24.5" × 15" (≈ 622 mm × 381 mm).
3. **Draw the grid on the board** (optional but recommended) using the above image as the single reference.  
   - The image shows the ArUco boundary (red), the internal grid (green, 4×4 by default), and the cell names (`R1C1` … `R4C4`).
   - For a 4×4 breakdown of a 24.5" × 15" area, each cell is ~6.125" wide × 3.75" tall (≈ 155.6 mm × 95.25 mm). Mark these dimensions on the board so the physical grid matches the camera overlay.

---

## 2. Streaming the Camera Feed With Bounds + Grid

Use the ArUco detection pipeline to display the workspace live:

```bash
python aruco_grid_visualizer.py --camera-index 0 --grid 4 4 --label-grid
```

- The script detects the four outer ArUco markers, draws the red bounding quadrilateral, and overlays the green grid with labels (`R1C1` etc.).
- When streaming, verify the overlay matches the physical grid drawn on the board. Adjust camera height/angle if the grid skews heavily.

Once the overlay looks correct, you can confirm the calibration and command moves directly from the live feed:

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

We'll capture a ground-truth pose and joint configuration for each cell to create a movement dictionary.

1. **Launch the manual capture tool:**

   ```bash
   python manual_pose_capture.py --output grid_samples.json --append
   ```

2. For each grid cell:
   - Type the grid name (e.g., `R2C3`).
   - The script disables torque; gently move the arm so the gripper is centered on that cell’s pickup point.
   - Re-enable torque by pressing Enter; the joints are recorded.
   - Enter the real-world coordinates (X_mm,Y_mm,Z_mm) for that cell center. You can read these from the camera overlay or the calibration map (`aruco_click_move.py`).
   - Repeat for all cells you care about.

3. `grid_samples.json` accumulates entries like:

   ```json
   [
     {
       "label": "R2C3",
       "coords_mm": { "x": 155.0, "y": 210.0, "z": 15.0 },
       "joints": {
         "shoulder_pan.pos": 2050,
         "shoulder_lift.pos": 2600,
         "elbow_flex.pos": 2400,
         "wrist_flex.pos": 1980,
         "wrist_roll.pos": 2200,
         "gripper.pos": 2176
       }
     }
   ]
   ```

Create additional JSON entries for:
- `shoulder_pan_bounds` (from `aruco_bounds_movement.json`)
- `shoulder_lift_bounds` (max up/down ticks)

---

## 4. Building the Movement Dictionary

Once the dataset is recorded:

1. Load `grid_samples.json` in a script (e.g., `grid_move_demo.py`):

   ```python
   import json
   from so101_control import connect_robot, move_to

   with open("grid_samples.json") as f:
       samples = {entry["label"]: entry for entry in json.load(f)}

   robot = connect_robot()
   move_to(robot, samples["R2C3"]["joints"])
   ```

2. Optionally store the dictionary in a dedicated file (e.g., `grid_pose_map.json`) mapping each grid name to joint targets + coordinates.

3. When the vision system spots an object in grid `RkCm`, it can:
   - Look up the pre-recorded pose in the dictionary.
   - Send the corresponding joint targets with `move_to`.
   - Close the gripper using the calibrated tick (`SQUARE_GRIPPER_TICKS`).

This eliminates guesswork: the robot always uses a validated pose for each cell.

---

## 5. Recommended Workflow

1. **Draw and label the grid** referencing `IMG_9051_grid_named.jpg`.
2. **Run the live detection script** (`aruco_grid_visualizer.py`) to verify the bounds overlay lines up with the physical grid.
3. **Capture robot poses** for each cell using `grid_pose_capture.py`, storing them in `grid_pose_map.json`.
4. **Use `aruco_click_move.py --live --connect`** to click any cell in the live feed and confirm the robot moves there using the saved poses.
5. **Integrate vision** so that when the system detects a knob at cell `R2C3`, it calls the corresponding pose and executes the pickup routine.

---

## Files in This Workflow

| File | Purpose |
| --- | --- |
| `aruco_grid_visualizer.py` | Shows the live camera feed with ArUco bounds and labeled grid to verify alignment before recording. |
| `grid_pose_capture.py` | Prompts you to move the robot to each grid cell (or the default pose), records joint angles and gripper ticks, and saves them into `grid_pose_map.json`. |
| `aruco_click_move.py` | Provides both static-image and live-camera click-to-move capability. Uses the calibration to convert pixels → XYZ and optionally command the robot to the clicked spot. |

### Robot Setup

1. Connect the SO-101 arm to your computer via the USB port defined in `so101_control.PORT`.
2. Ensure the robot is powered and that the Lerobot dependencies from `requirements.txt` are installed.
3. When running scripts that move the arm (`grid_pose_capture.py`, `aruco_click_move.py --connect`), keep the workspace clear and supervise the arm.

With these steps complete, placing an object at the center of any named grid square gives the robot a direct, pre-calibrated motion to pick it up.
