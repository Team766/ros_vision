#!/usr/bin/env python3
"""
Diagnostic tool for evaluating extrinsic calibration quality.

Reads a frameset directory, applies camera extrinsics to compute tag positions
in robot frame, and reports per-tag statistics + generates scatter plots.

For a perfect calibration on a rotating platform:
- Same tag seen by different cameras in the same frame should agree (small per-frame spread)
- Tag positions in robot frame will rotate across frames (expected), but the DISTANCE
  from robot center to each tag should be constant

Usage:
    python diagnose_extrinsics.py config.yaml

Config format (YAML):
    frameset_dir: "/path/to/calibration_data/"
    cameras:
      cam13:
        intrinsics_filename: "/path/to/calibrationmatrix_cam13.json"
        rotation:
          - [r00, r01, r02]
          - [r10, r11, r12]
          - [r20, r21, r22]
        offset: [x, y, z]
      cam14:
        intrinsics_filename: "/path/to/calibrationmatrix_cam14.json"
        rotation:
          - [r00, r01, r02]
          - [r10, r11, r12]
          - [r20, r21, r22]
        offset: [x, y, z]
"""

import argparse
import numpy as np
import cv2
import apriltag

try:
    from .utils import read_yaml_file
    from .solver import get_files, parse_filename
except ImportError:
    from utils import read_yaml_file
    from solver import get_files, parse_filename


def load_camera_extrinsics(config):
    """
    Load camera intrinsics and extrinsics from config.

    Returns dict of {cam_id: {intrinsic_matrix, rotation, offset}}.
    """
    cameras = {}
    for cam_id, cam_cfg in config["cameras"].items():
        intrinsic_data = read_yaml_file(cam_cfg["intrinsics_filename"])
        intrinsic_matrix = np.array(intrinsic_data["matrix"], dtype=np.float32)

        rotation = np.array(cam_cfg["rotation"], dtype=np.float64)
        offset = np.array(cam_cfg["offset"], dtype=np.float64)

        cameras[cam_id] = {
            "intrinsic_matrix": intrinsic_matrix,
            "rotation": rotation,
            "offset": offset,
        }
    return cameras


def detect_and_transform(config, cameras):
    """
    Detect AprilTags in all images, transform to robot frame using extrinsics.

    Returns list of observation dicts:
        {frame_num (int), cam_id, tag_id, pos_camera (3,), pos_robot (3,)}
    """
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)

    observations = []
    files = get_files(config["frameset_dir"])
    files.sort()

    for filename in files:
        frame = cv2.imread(filename)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grey)
        frame_num_str, cam_id = parse_filename(filename)
        frame_num = int(frame_num_str)

        cam = cameras[cam_id]
        m = cam["intrinsic_matrix"]
        params = (m[0, 0], m[1, 1], m[0, 2], m[1, 2])
        R = cam["rotation"]
        t = cam["offset"]

        for det in detections:
            solvedpose = detector.detection_pose(det, params, tag_size=0.1651)
            pos_camera = solvedpose[0][:3, 3]
            pos_robot = R @ pos_camera + t

            observations.append({
                "frame_num": frame_num,
                "cam_id": cam_id,
                "tag_id": det.tag_id,
                "pos_camera": pos_camera,
                "pos_robot": pos_robot,
            })

    print(f"Detected {len(observations)} observations across "
          f"{len(set(o['frame_num'] for o in observations))} frames")
    return observations


def compute_statistics(observations):
    """
    Compute per-tag statistics in robot frame.

    Reports:
    1. Per-tag position mean and std across all observations
    2. Per-tag distance from origin (robot center) mean and std
    3. Per-frame inter-camera disagreement for same tag
    """
    from collections import defaultdict

    # Group by tag
    by_tag = defaultdict(list)
    for obs in observations:
        by_tag[obs["tag_id"]].append(obs)

    print("\n" + "=" * 70)
    print("PER-TAG POSITION STATISTICS (robot frame)")
    print("=" * 70)

    tag_stats = {}
    for tag_id in sorted(by_tag.keys()):
        obs_list = by_tag[tag_id]
        positions = np.array([o["pos_robot"] for o in obs_list])
        distances = np.linalg.norm(positions, axis=1)

        mean_pos = positions.mean(axis=0)
        std_pos = positions.std(axis=0)
        mean_dist = distances.mean()
        std_dist = distances.std()

        tag_stats[tag_id] = {
            "positions": positions,
            "frames": [o["frame_num"] for o in obs_list],
            "cam_ids": [o["cam_id"] for o in obs_list],
            "mean_pos": mean_pos,
            "std_pos": std_pos,
            "mean_dist": mean_dist,
            "std_dist": std_dist,
            "n_obs": len(obs_list),
        }

        print(f"\nTag {tag_id} ({len(obs_list)} observations):")
        print(f"  Position mean: [{mean_pos[0]:.4f}, {mean_pos[1]:.4f}, {mean_pos[2]:.4f}]")
        print(f"  Position std:  [{std_pos[0]:.4f}, {std_pos[1]:.4f}, {std_pos[2]:.4f}]")
        print(f"  Distance from origin: mean={mean_dist:.4f}, std={std_dist:.4f}")

    # Per-frame inter-camera disagreement
    print("\n" + "=" * 70)
    print("PER-FRAME INTER-CAMERA DISAGREEMENT")
    print("=" * 70)

    by_frame_tag = defaultdict(list)
    for obs in observations:
        by_frame_tag[(obs["frame_num"], obs["tag_id"])].append(obs)

    disagreements = []
    for (frame_num, tag_id), obs_list in sorted(by_frame_tag.items()):
        if len(obs_list) >= 2:
            positions = np.array([o["pos_robot"] for o in obs_list])
            # Pairwise distance between camera observations
            diff = np.linalg.norm(positions[0] - positions[1])
            disagreements.append({
                "frame": frame_num,
                "tag_id": tag_id,
                "diff_m": diff,
            })

    if disagreements:
        diffs = np.array([d["diff_m"] for d in disagreements])
        print(f"\nTag pairs seen by both cameras: {len(disagreements)}")
        print(f"  Inter-camera disagreement (meters):")
        print(f"    Mean: {diffs.mean():.4f}")
        print(f"    Median: {np.median(diffs):.4f}")
        print(f"    Std: {diffs.std():.4f}")
        print(f"    Max: {diffs.max():.4f} (frame {disagreements[np.argmax(diffs)]['frame']}, "
              f"tag {disagreements[np.argmax(diffs)]['tag_id']})")
        print(f"    Min: {diffs.min():.4f}")
    else:
        print("\n  No tags seen by multiple cameras in same frame.")

    # Per-tag per-camera distance statistics
    print("\n" + "=" * 70)
    print("PER-TAG PER-CAMERA DISTANCE STATISTICS")
    print("=" * 70)
    print("(distance from robot center should be constant across frames)")

    by_tag_cam = defaultdict(list)
    for obs in observations:
        by_tag_cam[(obs["tag_id"], obs["cam_id"])].append(obs)

    all_distance_stds = []
    all_distance_ranges = []

    for (tag_id, cam_id) in sorted(by_tag_cam.keys()):
        obs_list = by_tag_cam[(tag_id, cam_id)]
        positions = np.array([o["pos_robot"] for o in obs_list])
        distances = np.linalg.norm(positions, axis=1)

        d_mean = distances.mean()
        d_std = distances.std()
        d_min = distances.min()
        d_max = distances.max()
        d_range = d_max - d_min

        all_distance_stds.append(d_std)
        all_distance_ranges.append(d_range)

        print(f"\n  Tag {tag_id}, Camera {cam_id} ({len(obs_list)} observations):")
        print(f"    Distance mean:  {d_mean * 100:.2f} cm")
        print(f"    Distance std:   {d_std * 100:.2f} cm")
        print(f"    Distance range: {d_range * 100:.2f} cm")
        print(f"    Distance min:   {d_min * 100:.2f} cm")
        print(f"    Distance max:   {d_max * 100:.2f} cm")

    all_distance_stds = np.array(all_distance_stds)
    all_distance_ranges = np.array(all_distance_ranges)

    print("\n" + "-" * 50)
    print("AGGREGATE DISTANCE STATISTICS (across all tag/camera pairs)")
    print("-" * 50)
    print(f"  Std  — mean: {all_distance_stds.mean() * 100:.2f} cm, "
          f"max: {all_distance_stds.max() * 100:.2f} cm, "
          f"min: {all_distance_stds.min() * 100:.2f} cm")
    print(f"  Range — mean: {all_distance_ranges.mean() * 100:.2f} cm, "
          f"max: {all_distance_ranges.max() * 100:.2f} cm, "
          f"min: {all_distance_ranges.min() * 100:.2f} cm")

    return tag_stats, disagreements


def plot_tag_positions(tag_stats, output_prefix="extrinsics_diag"):
    """
    Generate scatter plots of tag positions in robot frame.

    Creates:
    - XY plot (top-down view)
    - XZ plot (side view)
    - Distance-from-origin over frame number (per tag)
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available, skipping plots.")
        return

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(tag_stats), 1)))

    # XY scatter (top-down view)
    fig, ax = plt.subplots(figsize=(10, 8))
    for i, (tag_id, stats) in enumerate(sorted(tag_stats.items())):
        pos = stats["positions"]
        ax.scatter(pos[:, 0], pos[:, 1], s=8, alpha=0.5, color=colors[i % len(colors)],
                   label=f"Tag {tag_id} (n={stats['n_obs']})")
    ax.set_xlabel("X (m) - robot forward")
    ax.set_ylabel("Y (m) - robot left")
    ax.set_title("Tag Positions in Robot Frame (XY top-down)")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.plot(0, 0, "k+", markersize=15, markeredgewidth=2, label="Robot center")
    fname = f"{output_prefix}_xy.png"
    fig.savefig(fname, dpi=150, bbox_inches="tight")
    print(f"Saved: {fname}")
    plt.close(fig)

    # XZ scatter (side view)
    fig, ax = plt.subplots(figsize=(10, 8))
    for i, (tag_id, stats) in enumerate(sorted(tag_stats.items())):
        pos = stats["positions"]
        ax.scatter(pos[:, 0], pos[:, 2], s=8, alpha=0.5, color=colors[i % len(colors)],
                   label=f"Tag {tag_id}")
    ax.set_xlabel("X (m) - robot forward")
    ax.set_ylabel("Z (m) - robot up")
    ax.set_title("Tag Positions in Robot Frame (XZ side view)")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    fname = f"{output_prefix}_xz.png"
    fig.savefig(fname, dpi=150, bbox_inches="tight")
    print(f"Saved: {fname}")
    plt.close(fig)

    # Distance from origin over frame number (rotation invariant check)
    fig, ax = plt.subplots(figsize=(12, 6))
    for i, (tag_id, stats) in enumerate(sorted(tag_stats.items())):
        pos = stats["positions"]
        frames = np.array(stats["frames"])
        distances = np.linalg.norm(pos, axis=1)
        ax.scatter(frames, distances, s=8, alpha=0.5, color=colors[i % len(colors)],
                   label=f"Tag {tag_id} (std={stats['std_dist']:.4f}m)")
    ax.set_xlabel("Frame number")
    ax.set_ylabel("Distance from robot center (m)")
    ax.set_title("Tag Distance from Robot Center vs Frame\n"
                 "(should be flat if extrinsics are correct)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fname = f"{output_prefix}_distance.png"
    fig.savefig(fname, dpi=150, bbox_inches="tight")
    print(f"Saved: {fname}")
    plt.close(fig)

    # Per-tag XY with frame-number coloring (shows rotation arc)
    for tag_id, stats in sorted(tag_stats.items()):
        fig, ax = plt.subplots(figsize=(8, 8))
        pos = stats["positions"]
        frames = np.array(stats["frames"])
        sc = ax.scatter(pos[:, 0], pos[:, 1], c=frames, s=15, alpha=0.7, cmap="viridis")
        plt.colorbar(sc, ax=ax, label="Frame number")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(f"Tag {tag_id} Position in Robot Frame (colored by frame)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.plot(0, 0, "r+", markersize=15, markeredgewidth=2)
        fname = f"{output_prefix}_tag{tag_id}_xy.png"
        fig.savefig(fname, dpi=150, bbox_inches="tight")
        plt.close(fig)

    print(f"Saved per-tag XY plots for {len(tag_stats)} tags")


def write_markdown_report(
    cameras, tag_stats, disagreements, observations, output_path, plot_prefix
):
    """
    Write a markdown report with formatted tables summarizing calibration accuracy.
    """
    from collections import defaultdict

    lines = []
    lines.append("# Extrinsic Calibration Accuracy Report\n")

    # Camera extrinsics summary
    lines.append("## Camera Extrinsics\n")
    lines.append("| Camera | Offset X (m) | Offset Y (m) | Offset Z (m) |")
    lines.append("|--------|-------------|-------------|-------------|")
    for cam_id in sorted(cameras.keys()):
        o = cameras[cam_id]["offset"]
        lines.append(f"| {cam_id} | {o[0]:.4f} | {o[1]:.4f} | {o[2]:.4f} |")
    lines.append("")

    # Per-tag position statistics
    lines.append("## Per-Tag Position Statistics (robot frame)\n")
    lines.append("| Tag | Obs | Mean X (m) | Mean Y (m) | Mean Z (m) | "
                 "Std X (cm) | Std Y (cm) | Std Z (cm) | Dist Mean (m) | Dist Std (cm) |")
    lines.append("|-----|-----|-----------|-----------|-----------|"
                 "-----------|-----------|-----------|--------------|--------------|")
    for tag_id in sorted(tag_stats.keys()):
        s = tag_stats[tag_id]
        mp = s["mean_pos"]
        sp = s["std_pos"]
        lines.append(
            f"| {tag_id} | {s['n_obs']} | {mp[0]:.4f} | {mp[1]:.4f} | {mp[2]:.4f} | "
            f"{sp[0]*100:.2f} | {sp[1]*100:.2f} | {sp[2]*100:.2f} | "
            f"{s['mean_dist']:.4f} | {s['std_dist']*100:.2f} |"
        )
    lines.append("")

    # Per-tag per-camera distance statistics
    lines.append("## Per-Tag Per-Camera Distance Statistics\n")
    lines.append("Distance from robot center should be constant across frames "
                 "if extrinsics are correct.\n")
    lines.append("| Tag | Camera | Obs | Mean (cm) | Std (cm) | Range (cm) | Min (cm) | Max (cm) |")
    lines.append("|-----|--------|-----|----------|---------|-----------|---------|---------|")

    by_tag_cam = defaultdict(list)
    for obs in observations:
        by_tag_cam[(obs["tag_id"], obs["cam_id"])].append(obs)

    all_stds = []
    all_ranges = []
    for (tag_id, cam_id) in sorted(by_tag_cam.keys()):
        obs_list = by_tag_cam[(tag_id, cam_id)]
        positions = np.array([o["pos_robot"] for o in obs_list])
        distances = np.linalg.norm(positions, axis=1)
        d_mean = distances.mean()
        d_std = distances.std()
        d_min = distances.min()
        d_max = distances.max()
        d_range = d_max - d_min
        all_stds.append(d_std)
        all_ranges.append(d_range)
        lines.append(
            f"| {tag_id} | {cam_id} | {len(obs_list)} | "
            f"{d_mean*100:.2f} | {d_std*100:.2f} | {d_range*100:.2f} | "
            f"{d_min*100:.2f} | {d_max*100:.2f} |"
        )
    lines.append("")

    # Aggregate distance statistics
    all_stds = np.array(all_stds)
    all_ranges = np.array(all_ranges)
    lines.append("## Aggregate Distance Statistics\n")
    lines.append("| Metric | Mean (cm) | Max (cm) | Min (cm) |")
    lines.append("|--------|----------|---------|---------|")
    lines.append(f"| Std | {all_stds.mean()*100:.2f} | {all_stds.max()*100:.2f} | "
                 f"{all_stds.min()*100:.2f} |")
    lines.append(f"| Range | {all_ranges.mean()*100:.2f} | {all_ranges.max()*100:.2f} | "
                 f"{all_ranges.min()*100:.2f} |")
    lines.append("")

    # Inter-camera disagreement
    if disagreements:
        diffs = np.array([d["diff_m"] for d in disagreements])
        lines.append("## Inter-Camera Disagreement\n")
        lines.append(f"Tag pairs seen by both cameras: {len(disagreements)}\n")
        lines.append("| Metric | Value (cm) |")
        lines.append("|--------|-----------|")
        lines.append(f"| Mean | {diffs.mean()*100:.2f} |")
        lines.append(f"| Median | {np.median(diffs)*100:.2f} |")
        lines.append(f"| Std | {diffs.std()*100:.2f} |")
        lines.append(f"| Max | {diffs.max()*100:.2f} |")
        lines.append(f"| Min | {diffs.min()*100:.2f} |")
        lines.append("")

    # Embedded plot references
    lines.append("## Plots\n")
    lines.append(f"![XY Top-Down]({plot_prefix}_xy.png)\n")
    lines.append(f"![XZ Side View]({plot_prefix}_xz.png)\n")
    lines.append(f"![Distance vs Frame]({plot_prefix}_distance.png)\n")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))

    print(f"Markdown report written to: {output_path}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Diagnose extrinsic calibration quality from frameset data."
    )
    parser.add_argument("config", type=str, help="Path to diagnostic config YAML file")
    parser.add_argument("--no-plot", action="store_true", help="Skip generating plots")
    parsed_args = parser.parse_args(args)

    config = read_yaml_file(parsed_args.config)
    cameras = load_camera_extrinsics(config)

    # Auto-generate output prefix from camera IDs
    cam_ids = sorted(cameras.keys())
    prefix = "_".join(cam_ids)
    plot_prefix = prefix + "_diag"
    report_path = prefix + "_accuracy.md"

    print("Loaded extrinsics:")
    for cam_id, cam in cameras.items():
        print(f"  {cam_id}: offset={cam['offset'].tolist()}")

    observations = detect_and_transform(config, cameras)
    tag_stats, disagreements = compute_statistics(observations)

    if not parsed_args.no_plot:
        plot_tag_positions(tag_stats, output_prefix=plot_prefix)

    write_markdown_report(
        cameras, tag_stats, disagreements, observations, report_path, plot_prefix
    )


if __name__ == "__main__":
    main()
