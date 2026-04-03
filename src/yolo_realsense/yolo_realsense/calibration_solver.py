"""
calibration_solver.py
=====================
Step 2 of 2 — Solve for the calibration matrix from collected points.

HOW TO USE:
1. Run calibration_collector.py first to generate calibration_points.npz
2. Run this script:   python3 calibration_solver.py
3. It prints the matrix M and saves it to calibration_matrix.npz
4. Paste the printed M into yolo_distance_node.py (instructions printed too)
"""

import numpy as np
import os


def solve_calibration(points_path: str, output_path: str):
    # --- Load points ---
    if not os.path.exists(points_path):
        print(f"ERROR: {points_path} not found. Run calibration_collector.py first.")
        return

    data = np.load(points_path)
    cam_pts = data['camera_pts']      # Shape: Nx3, in meters
    gantry_pts = data['gantry_pts']   # Shape: Nx3, in mm

    n = len(cam_pts)
    print(f"\nLoaded {n} calibration points.")
    print("\nRaw points:")
    print(f"{'#':<4} {'Cam X':>10} {'Cam Y':>10} {'Cam Z':>10}    {'Gantry X':>10} {'Gantry Y':>10} {'Gantry Z':>10}")
    print("-" * 70)
    for i, (c, g) in enumerate(zip(cam_pts, gantry_pts)):
        print(f"{i:<4} {c[0]:>10.4f} {c[1]:>10.4f} {c[2]:>10.4f}    {g[0]:>10.1f} {g[1]:>10.1f} {g[2]:>10.1f}")

    if n < 4:
        print(f"\nERROR: Need at least 4 points, only have {n}.")
        return

    # --- Build homogeneous camera matrix [cam_x, cam_y, cam_z, 1] ---
    ones = np.ones((n, 1))
    cam_h = np.hstack([cam_pts, ones])   # Shape: Nx4

    # --- Solve: gantry = cam_h @ M  (least squares) ---
    # M is 4x3: [scale+rotation rows for x,y,z] + [translation row]
    M, residuals, rank, sv = np.linalg.lstsq(cam_h, gantry_pts, rcond=None)

    # --- Evaluate fit ---
    predicted = cam_h @ M
    errors = gantry_pts - predicted
    per_point_error = np.sqrt(np.sum(errors ** 2, axis=1))

    print(f"\n{'='*60}")
    print("CALIBRATION RESULTS")
    print(f"{'='*60}")
    print(f"\nMatrix rank: {rank} (should be 4 for full rank)")
    print(f"\nPer-point errors (mm):")
    print(f"{'#':<4} {'Error (mm)':>12}  {'Pred X':>10} {'Pred Y':>10} {'Pred Z':>10}  |  {'True X':>10} {'True Y':>10} {'True Z':>10}")
    print("-" * 90)
    for i, (err, pred, true) in enumerate(zip(per_point_error, predicted, gantry_pts)):
        flag = " <-- HIGH ERROR, consider re-collecting" if err > 5.0 else ""
        print(f"{i:<4} {err:>12.2f}  {pred[0]:>10.1f} {pred[1]:>10.1f} {pred[2]:>10.1f}  |  {true[0]:>10.1f} {true[1]:>10.1f} {true[2]:>10.1f}{flag}")

    print(f"\nSummary:")
    print(f"  Max error:  {per_point_error.max():.2f} mm")
    print(f"  Mean error: {per_point_error.mean():.2f} mm")
    print(f"  RMS error:  {np.sqrt(np.mean(per_point_error**2)):.2f} mm")

    if per_point_error.max() > 10.0:
        print("\nWARNING: Max error > 10mm. Consider:")
        print("  - Re-collecting points with high error (marked above)")
        print("  - Adding more points spread across the workspace")
        print("  - Checking that gantry positions were recorded accurately")
    elif per_point_error.max() > 5.0:
        print("\nOK: Accuracy is acceptable but could be improved.")
        print("  Consider adding more points in underrepresented areas.")
    else:
        print("\nEXCELLENT: Calibration looks great!")

    # --- Save matrix ---
    np.savez(output_path, M=M)
    print(f"\nCalibration matrix saved to: {output_path}")

    # --- Print matrix for copy-paste into node ---
    print(f"\n{'='*60}")
    print("PASTE THIS INTO yolo_distance_node.py:")
    print(f"{'='*60}")
    print("\nself.cam_to_gantry_M = np.array([")
    for row in M:
        print(f"    [{row[0]:>14.6f}, {row[1]:>14.6f}, {row[2]:>14.6f}],")
    print("])")
    print()
    print("# Errors: mean={:.2f}mm  max={:.2f}mm  points={}".format(
        per_point_error.mean(), per_point_error.max(), n))
    print(f"{'='*60}\n")

    return M


if __name__ == '__main__':
    home = os.path.expanduser('~')
    points_path = os.path.join(home, 'calibration_points.npz')
    output_path = os.path.join(home, 'calibration_matrix.npz')
    solve_calibration(points_path, output_path)
