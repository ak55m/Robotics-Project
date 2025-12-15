"""
Live ArUco boundary + grid visualizer.

This script detects the four outer ArUco markers, draws the bounding quadrilateral,
picks a uniform MxN grid inside that region, and optionally labels each cell.
Use it to verify that your physical grid matches the camera view before collecting
robot calibration samples.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class GridConfig:
    rows: int
    cols: int
    label_cells: bool
    line_color: Tuple[int, int, int] = (0, 255, 0)
    label_color: Tuple[int, int, int] = (255, 0, 0)
    boundary_color: Tuple[int, int, int] = (0, 0, 255)


def detect_corner_markers(
    image: np.ndarray, dictionary: int
) -> List[np.ndarray] | None:
    """Return the four largest marker corner sets (outer tags)."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    params = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or len(corners) < 4:
        return None

    areas = [cv2.contourArea(corner[0].astype(np.float32)) for corner in corners]
    idx = np.argsort(areas)[::-1][:4]
    return [corners[i][0] for i in idx]


def order_corners(corners: List[np.ndarray]) -> np.ndarray:
    """Return corners ordered TL, TR, BR, BL."""
    centers = np.array([corner.mean(axis=0) for corner in corners])
    sums = centers.sum(axis=1)
    diffs = np.diff(centers, axis=1).flatten()

    tl = centers[np.argmin(sums)]
    br = centers[np.argmax(sums)]
    tr = centers[np.argmin(diffs)]
    bl = centers[np.argmax(diffs)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def draw_grid(
    frame: np.ndarray,
    quad: np.ndarray,
    config: GridConfig,
) -> None:
    """Overlay boundary + grid and optional labels onto the frame."""
    cv2.polylines(frame, [quad.astype(np.int32)], True, config.boundary_color, 3)
    tl, tr, br, bl = quad

    # Grid lines
    for i in range(1, config.cols):
        t = i / config.cols
        p1 = ((1 - t) * tl + t * tr).astype(int)
        p2 = ((1 - t) * bl + t * br).astype(int)
        cv2.line(frame, tuple(p1), tuple(p2), config.line_color, 2)
    for j in range(1, config.rows):
        t = j / config.rows
        p1 = ((1 - t) * tl + t * bl).astype(int)
        p2 = ((1 - t) * tr + t * br).astype(int)
        cv2.line(frame, tuple(p1), tuple(p2), config.line_color, 2)

    if not config.label_cells:
        return

    font = cv2.FONT_HERSHEY_SIMPLEX
    for r in range(config.rows):
        for c in range(config.cols):
            u = (c + 0.5) / config.cols
            v = (r + 0.5) / config.rows
            top_interp = (1 - u) * tl + u * tr
            bottom_interp = (1 - u) * bl + u * br
            center = (1 - v) * top_interp + v * bottom_interp
            label = f"R{r+1}C{c+1}"
            cv2.putText(
                frame,
                label,
                tuple(center.astype(int)),
                font,
                0.6,
                config.label_color,
                2,
                cv2.LINE_AA,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Live ArUco bounds + grid visualizer.")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index")
    parser.add_argument(
        "--grid",
        type=int,
        nargs=2,
        metavar=("ROWS", "COLS"),
        default=(4, 4),
        help="Grid subdivision (rows cols). Default 4 4.",
    )
    parser.add_argument(
        "--label-grid",
        action="store_true",
        help="Write R#C# labels at each grid cell center.",
    )
    parser.add_argument(
        "--dictionary",
        type=str,
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary name (default: DICT_4X4_50).",
    )
    args = parser.parse_args()

    dictionary = getattr(cv2.aruco, args.dictionary, None)
    if dictionary is None:
        print(f"❌ Unknown dictionary {args.dictionary}")
        sys.exit(1)

    config = GridConfig(rows=args.grid[0], cols=args.grid[1], label_cells=args.label_grid)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        print(f"❌ Could not open camera {args.camera_index}")
        sys.exit(1)

    print("Press 'q' to quit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Failed to read frame")
                break

            corners = detect_corner_markers(frame, dictionary)
            if corners and len(corners) == 4:
                quad = order_corners(corners)
                draw_grid(frame, quad, config)
            else:
                cv2.putText(
                    frame,
                    "Waiting for 4 corner markers...",
                    (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("ArUco Grid Visualizer", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
