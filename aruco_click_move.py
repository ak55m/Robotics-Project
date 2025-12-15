"""
Auto-calibrate from four ArUco corner markers (with known physical dimensions),
then let the user click anywhere in the image to command the SO-101 robot.

The pipeline:
    1. Detect the 4 corner markers inside the provided image frame.
    2. Map them to the real-world rectangle dimensions (width_mm × height_mm).
    3. Build a CameraToRobotCalibrator so every pixel converts to millimeters.
    4. On each left-click, convert the pixel to XYZ and (optionally) move the robot.

Example:
    python aruco_click_move.py --image IMG_9046.jpg --connect \
        --width-mm 622.3 --height-mm 381.0 --z-mm 25
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from camera_robot_bridge import CalibrationPair, CameraToRobotCalibrator, RobotMotionController
from so101_control import PORT, connect_robot


def detect_corner_markers(image: np.ndarray, dictionary: int) -> List[Tuple[float, float]]:
    """Return the four corner centers sorted TL, TR, BR, BL."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    parameters = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if corners is None or len(corners) < 4:
        raise RuntimeError(f"Found {0 if corners is None else len(corners)} markers, need 4.")

    centers = []
    for corner in corners:
        pts = corner[0]
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        centers.append((cx, cy))

    centers_array = np.array(centers)
    s = centers_array.sum(axis=1)
    diff = np.diff(centers_array, axis=1)

    top_left = centers_array[np.argmin(s)]
    bottom_right = centers_array[np.argmax(s)]
    top_right = centers_array[np.argmin(diff)]
    bottom_left = centers_array[np.argmax(diff)]

    return [
        tuple(top_left),
        tuple(top_right),
        tuple(bottom_right),
        tuple(bottom_left),
    ]


def build_calibrator_from_rectangle(
    corner_pixels: List[Tuple[float, float]], width_mm: float, height_mm: float, default_z: float
) -> CameraToRobotCalibrator:
    """Construct a calibrator given TL/TR/BR/BL pixel points."""
    mm_targets = [
        (0.0, 0.0),  # top-left
        (width_mm, 0.0),  # top-right
        (width_mm, height_mm),  # bottom-right
        (0.0, height_mm),  # bottom-left
    ]
    samples = [
        CalibrationPair(pixel=pix, robot_xy=mm) for pix, mm in zip(corner_pixels, mm_targets)
    ]
    calibrator = CameraToRobotCalibrator(default_z=default_z)
    calibrator.calibrate(samples)
    return calibrator


def quad_from_centers(centers: List[Tuple[float, float]]) -> np.ndarray:
    centers_array = np.array(centers)
    s = centers_array.sum(axis=1)
    diff = np.diff(centers_array, axis=1)
    top_left = centers_array[np.argmin(s)]
    bottom_right = centers_array[np.argmax(s)]
    top_right = centers_array[np.argmin(diff)]
    bottom_left = centers_array[np.argmax(diff)]
    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)


def draw_grid_overlay(
    frame: np.ndarray,
    quad: np.ndarray,
    rows: int,
    cols: int,
    label_cells: bool,
) -> None:
    cv2.polylines(frame, [quad.astype(np.int32)], True, (0, 0, 255), 2)
    tl, tr, br, bl = quad
    for i in range(1, cols):
        t = i / cols
        p1 = ((1 - t) * tl + t * tr).astype(int)
        p2 = ((1 - t) * bl + t * br).astype(int)
        cv2.line(frame, tuple(p1), tuple(p2), (0, 255, 0), 2)
    for j in range(1, rows):
        t = j / rows
        p1 = ((1 - t) * tl + t * bl).astype(int)
        p2 = ((1 - t) * tr + t * br).astype(int)
        cv2.line(frame, tuple(p1), tuple(p2), (0, 255, 0), 2)
    if not label_cells:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    for r in range(rows):
        for c in range(cols):
            u = (c + 0.5) / cols
            v = (r + 0.5) / rows
            top_interp = (1 - u) * tl + u * tr
            bottom_interp = (1 - u) * bl + u * br
            center = (1 - v) * top_interp + v * bottom_interp
            cv2.putText(
                frame,
                f"R{r+1}C{c+1}",
                tuple(center.astype(int)),
                font,
                0.6,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Click-to-move demo using ArUco auto-calibration.")
    parser.add_argument("--image", type=Path, help="Captured calibration image.")
    parser.add_argument(
        "--live",
        action="store_true",
        help="Use live camera feed (requires webcam) instead of a still image.",
    )
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index for live mode.")
    parser.add_argument(
        "--width-mm",
        type=float,
        default=622.3,
        help="Long side length in millimeters (default 24.5 in).",
    )
    parser.add_argument(
        "--height-mm",
        type=float,
        default=381.0,
        help="Short side length in millimeters (default 15 in).",
    )
    parser.add_argument(
        "--z-mm",
        type=float,
        default=0.0,
        help="Target Z height for motions (relative to robot base).",
    )
    parser.add_argument("--wrist-roll", type=float, default=0.0, help="Wrist roll angle (deg).")
    parser.add_argument(
        "--gripper-pos",
        type=float,
        default=None,
        help="Optional gripper target value to send with each move.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        help="Existing calibration JSON (skips on-the-fly solve).",
    )
    parser.add_argument(
        "--connect",
        action="store_true",
        help=f"Connect to the robot on {PORT} and move on each click.",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=5.0,
        help="Seconds to allow for each move_to command (higher = slower motion).",
    )
    parser.add_argument(
        "--save-calibration",
        type=Path,
        help="Optional path to save the derived calibration JSON.",
    )
    parser.add_argument(
        "--grid-rows",
        type=int,
        default=4,
        help="Rows for overlay grid (display only).",
    )
    parser.add_argument(
        "--grid-cols",
        type=int,
        default=4,
        help="Columns for overlay grid (display only).",
    )
    parser.add_argument(
        "--label-grid",
        action="store_true",
        help="Label grid cells in the overlay.",
    )
    parser.add_argument(
        "--dictionary",
        type=str,
        default="DICT_4X4_1000",
        help="OpenCV ArUco dictionary constant (default DICT_4X4_1000).",
    )
    args = parser.parse_args()

    dictionary = getattr(cv2.aruco, args.dictionary, None)
    if dictionary is None:
        raise SystemExit(f"Unknown ArUco dictionary: {args.dictionary}")

    if not args.live and args.image is None:
        parser.error("Provide --image for static mode or use --live for camera mode.")

    controller = RobotMotionController()
    robot = None

    if args.connect:
        print(f"\nConnecting to robot on {PORT} ...")
        robot = connect_robot(PORT)
        controller.robot = robot
        print("✓ Connected.")

    def process_clicks(click_points, calibrator_obj, frame=None):
        while click_points:
            x, y = click_points.pop(0)
            if calibrator_obj is None:
                print("⚠️  Calibration not ready yet.")
                continue
            target_xyz = calibrator_obj.pixel_to_robot_xyz((x, y), args.z_mm)
            print(f"\nClicked pixel ({x}, {y}) -> target XYZ {target_xyz}")
            controller.execute_cartesian_move(
                target_xyz,
                wrist_roll=args.wrist_roll,
                gripper_pos=args.gripper_pos,
                duration=args.move_duration,
            )
            if frame is not None:
                cv2.circle(frame, (x, y), 6, (0, 255, 0), -1)

    if args.live:
        cap = cv2.VideoCapture(args.camera_index)
        if not cap.isOpened():
            raise SystemExit(f"Could not open camera {args.camera_index}")

        cv2.namedWindow("ArUco Click-to-Move")
        click_points: List[Tuple[int, int]] = []
        calibrator: Optional[CameraToRobotCalibrator] = (
            CameraToRobotCalibrator.from_file(args.calibration)
            if args.calibration
            else None
        )
        saved = False

        def on_click(event, x, y, flags, userdata):
            if event == cv2.EVENT_LBUTTONDOWN:
                click_points.append((x, y))

        cv2.setMouseCallback("ArUco Click-to-Move", on_click)
        print("\nLive mode:")
        print("  - Left-click to command moves once calibration is ready.")
        print("  - Press 'q' to quit.")

        last_quad = None
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("⚠️  Failed to read frame.")
                    break

                try:
                    centers = detect_corner_markers(frame, dictionary)
                    quad = quad_from_centers(centers)
                    last_quad = quad
                    draw_grid_overlay(
                        frame, quad, args.grid_rows, args.grid_cols, args.label_grid
                    )
                    if calibrator is None:
                        calibrator = build_calibrator_from_rectangle(
                            centers, args.width_mm, args.height_mm, args.z_mm
                        )
                        print("✓ Calibration computed from live frame.")
                        if args.save_calibration and not saved:
                            calibrator.save(args.save_calibration)
                            print(f"Saved calibration -> {args.save_calibration}")
                            saved = True
                except RuntimeError:
                    if last_quad is None:
                        cv2.putText(
                            frame,
                            "Waiting for 4 corner markers...",
                            (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2,
                        )

                process_clicks(click_points, calibrator, frame)
                cv2.imshow("ArUco Click-to-Move", frame)
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
    else:
        image = cv2.imread(str(args.image))
        if image is None:
            raise SystemExit(f"Could not load image: {args.image}")

        calibrator: Optional[CameraToRobotCalibrator]
        if args.calibration:
            calibrator = CameraToRobotCalibrator.from_file(args.calibration)
        else:
            centers = detect_corner_markers(image, dictionary)
            calibrator = build_calibrator_from_rectangle(
                centers, args.width_mm, args.height_mm, args.z_mm
            )
            print("Detected corner centers (TL, TR, BR, BL):")
            for label, pt in zip(["TL", "TR", "BR", "BL"], centers):
                print(f"  {label}: ({pt[0]:.1f}, {pt[1]:.1f}) px")
            if args.save_calibration:
                calibrator.save(args.save_calibration)
                print(f"Saved calibration -> {args.save_calibration}")
        display = image.copy()
        try:
            centers = detect_corner_markers(image, dictionary)
            quad = quad_from_centers(centers)
            draw_grid_overlay(display, quad, args.grid_rows, args.grid_cols, args.label_grid)
        except RuntimeError:
            pass

        click_points: List[Tuple[int, int]] = []

        def on_click_img(event, x, y, flags, userdata):
            if event == cv2.EVENT_LBUTTONDOWN:
                click_points.append((x, y))

        cv2.namedWindow("ArUco Click-to-Move")
        cv2.setMouseCallback("ArUco Click-to-Move", on_click_img)
        print("\nStatic image mode:")
        print("  - Left-click to command moves.")
        print("  - Press 'q' to quit.")
        while True:
            process_clicks(click_points, calibrator, display)
            cv2.imshow("ArUco Click-to-Move", display)
            if cv2.waitKey(20) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()

    if robot is not None:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
