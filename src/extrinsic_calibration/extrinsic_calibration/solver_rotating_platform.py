#!/usr/bin/env python3
"""
Extrinsic calibration solver for a rotating platform (turntable) setup.

The robot sits on a turntable surrounded by fixed AprilTags. As the robot
rotates, its center stays fixed. This solver jointly optimizes:
  - Camera extrinsic parameters (rotation + translation relative to robot center)
  - Per-frame turntable angle (rotation about Z axis)
  - Per-tag world positions
  - Robot center position in world frame

The key constraint: every observation of a given tag, across all frames and
cameras, must predict the same world position after accounting for the
per-frame turntable rotation.

For observation (frame f, camera i, tag j, measured position p_j^ci):
    p_predicted = R_z(theta_f) @ (R_cam_i @ p_j^ci + t_cam_i) + p_robot

Loss = mean( ||p_predicted - T_j||^2 ) over all observations.

Usage:
    python solver_rotating_platform.py config.yaml
"""

import argparse
import json
import numpy as np
import torch
import torch.optim as optim
import yaml
from tqdm import trange

# Try to import from local utils, fallback to standalone utils
try:
    from .utils import (
        read_yaml_file,
        compose_rotations_xyz_torch,
        camera_to_robot_torch,
    )
    from .solver import (
        get_device,
        get_files,
        parse_filename,
        draw_tag,
        setup_cameras,
        create_torch_params,
    )
except ImportError:
    from utils import read_yaml_file, compose_rotations_xyz_torch, camera_to_robot_torch
    from solver import (
        get_device,
        get_files,
        parse_filename,
        draw_tag,
        setup_cameras,
        create_torch_params,
    )

import apriltag
import cv2


def generate_frameset_all_observations(config, cams_dict):
    """
    Detect AprilTags in all images and return every observation (not just pairs).

    Unlike the original generate_frameset which only keeps tags seen by exactly
    2 cameras, this keeps ALL observations so that cross-frame constraints
    can be applied.

    Args:
        config: Configuration dict with 'frameset_dir'.
        cams_dict: Camera info dict with intrinsic matrices.

    Returns:
        tuple: (observations, frame_ids, tag_ids)
            - observations: list of dicts with keys:
                frame_idx, frame_num, cam_id, tag_id, translation
            - frame_ids: sorted list of unique frame number strings
            - tag_ids: sorted list of unique tag IDs (ints)
    """
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)

    # First pass: collect all raw observations
    raw_observations = []
    files = get_files(config["frameset_dir"])
    files.sort()

    for filename in files:
        frame = cv2.imread(filename)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection_list = detector.detect(grey)
        frame_num, camid = parse_filename(filename)

        for det in detection_list:
            m = cams_dict[camid]["intrinsic_matrix"]
            params = (m[0, 0], m[1, 1], m[0, 2], m[1, 2])
            solvedpose = detector.detection_pose(det, params, tag_size=0.1651)
            translation = solvedpose[0][:3, 3]

            raw_observations.append(
                {
                    "frame_num": frame_num,
                    "cam_id": camid,
                    "tag_id": det.tag_id,
                    "translation": translation,
                }
            )
            draw_tag(frame, det)

    # Build sorted ID lists and index mappings (numeric sort for frame numbers)
    frame_ids = sorted(set(obs["frame_num"] for obs in raw_observations), key=int)
    tag_ids = sorted(set(obs["tag_id"] for obs in raw_observations))
    frame_id_to_idx = {fid: i for i, fid in enumerate(frame_ids)}
    tag_id_to_idx = {tid: i for i, tid in enumerate(tag_ids)}

    # Second pass: add indices
    observations = []
    for obs in raw_observations:
        observations.append(
            {
                "frame_idx": frame_id_to_idx[obs["frame_num"]],
                "frame_num": obs["frame_num"],
                "cam_id": obs["cam_id"],
                "tag_id": obs["tag_id"],
                "tag_idx": tag_id_to_idx[obs["tag_id"]],
                "translation": obs["translation"],
            }
        )

    print(
        f"Detected {len(observations)} total observations across "
        f"{len(frame_ids)} frames, {len(tag_ids)} unique tags, "
        f"{len(cams_dict)} cameras"
    )

    return observations, frame_ids, tag_ids


def preprocess_observations(observations, cam_ids, device):
    """
    Convert observations into batched GPU tensors for efficient loss computation.

    Args:
        observations: List of observation dicts from generate_frameset_all_observations.
        cam_ids: List of camera ID strings.
        device: Torch device.

    Returns:
        dict with:
            coords: (N, 3) tensor of tag positions in camera frame
            frame_indices: (N,) int tensor of frame indices
            tag_indices: (N,) int tensor of tag indices
            cam_masks: {cam_id: (M,) index tensor} for per-camera batched ops
            num_obs, num_frames, num_tags: counts
    """
    N = len(observations)
    coords = np.array([obs["translation"] for obs in observations], dtype=np.float32)
    frame_indices = np.array([obs["frame_idx"] for obs in observations], dtype=np.int64)
    tag_indices = np.array([obs["tag_idx"] for obs in observations], dtype=np.int64)

    # Per-camera index masks
    cam_masks = {}
    for cam_id in cam_ids:
        idx = [i for i, obs in enumerate(observations) if obs["cam_id"] == cam_id]
        if idx:
            cam_masks[cam_id] = torch.tensor(idx, dtype=torch.long, device=device)

    num_frames = max(frame_indices) + 1 if N > 0 else 0
    num_tags = max(tag_indices) + 1 if N > 0 else 0

    return {
        "coords": torch.tensor(coords, dtype=torch.float32, device=device),
        "frame_indices": torch.tensor(frame_indices, dtype=torch.long, device=device),
        "tag_indices": torch.tensor(tag_indices, dtype=torch.long, device=device),
        "cam_masks": cam_masks,
        "num_obs": N,
        "num_frames": int(num_frames),
        "num_tags": int(num_tags),
    }


def initialize_tag_positions(batch, torch_params, device):
    """
    Compute initial tag world positions by transforming the first observation
    of each tag through its camera's extrinsics.

    Since all frame angles initialize to 0, these positions are approximate
    regardless of which frame they come from. The optimizer will refine them.

    Args:
        batch: Preprocessed batch data.
        torch_params: Camera parameter tensors.
        device: Torch device.

    Returns:
        (T, 3) tensor of initial tag world positions.
    """
    num_tags = batch["num_tags"]
    tag_positions = torch.zeros(num_tags, 3, dtype=torch.float32, device=device)
    tag_initialized = [False] * num_tags

    coords = batch["coords"]
    tag_indices = batch["tag_indices"]

    # Use the first observation of each tag (from any frame/camera)
    for cam_id, idx_tensor in batch["cam_masks"].items():
        angles = torch_params[cam_id]["rotations"]
        trans = torch_params[cam_id]["translation"]
        R_cam = compose_rotations_xyz_torch(
            angles[0], angles[1], angles[2]
        ) @ camera_to_robot_torch(device=device)

        for i in idx_tensor.tolist():
            tidx = tag_indices[i].item()
            if not tag_initialized[tidx]:
                c = coords[i]
                world_pos = R_cam @ c + trans
                tag_positions[tidx] = world_pos.detach()
                tag_initialized[tidx] = True

    return tag_positions


def compute_loss(
    batch,
    torch_params,
    frame_angles_free,
    tag_world_positions,
    robot_position,
    initial_translations,
    device,
):
    """
    Compute MSE loss for the rotating platform constraint.

    For each observation: predicted_world = R_z(theta_f) @ (R_cam @ p_cam + t_cam) + p_robot
    Loss = mean(||predicted_world - T_tag||^2) + regularization on camera translations

    The regularization prevents the solver from exploiting the translational
    gauge freedom (shifting all t_cam and tag positions together).

    Args:
        batch: Preprocessed observation batch.
        torch_params: Camera extrinsic parameters.
        frame_angles_free: (F-1,) tensor of turntable angles for frames 1..F-1 (degrees).
        tag_world_positions: (T, 3) tensor of tag world positions.
        robot_position: (3,) tensor of robot center position.
        initial_translations: dict of {cam_id: (3,) tensor} initial camera translations.
        device: Torch device.

    Returns:
        Scalar loss tensor.
    """
    coords = batch["coords"]  # (N, 3)
    frame_idx = batch["frame_indices"]  # (N,)
    tag_idx = batch["tag_indices"]  # (N,)

    # Step 1: Transform from camera frame to robot frame (per-camera batched)
    robot_coords = torch.empty_like(coords)
    for cam_id in batch["cam_masks"]:
        idx = batch["cam_masks"][cam_id]
        angles = torch_params[cam_id]["rotations"]
        trans = torch_params[cam_id]["translation"]
        R_cam = compose_rotations_xyz_torch(
            angles[0], angles[1], angles[2]
        ) @ camera_to_robot_torch(device=device)

        c = coords[idx]  # (M, 3)
        robot_coords[idx] = (R_cam @ c.mT).mT + trans

    # Step 2: Build full frame angles vector with theta_0 = 0 (fixed)
    zero = torch.zeros(1, dtype=torch.float32, device=device)
    frame_angles = torch.cat([zero, frame_angles_free])  # (F,)

    # Gather per-observation turntable angles
    thetas_deg = frame_angles[frame_idx]  # (N,)
    theta_rad = thetas_deg * (torch.pi / 180.0)

    # Step 3: Batched R_z rotation
    c = torch.cos(theta_rad)  # (N,)
    s = torch.sin(theta_rad)  # (N,)
    zeros = torch.zeros_like(c)
    ones = torch.ones_like(c)

    # Build (N, 3, 3) rotation matrices
    R_z = torch.stack(
        [
            torch.stack([c, -s, zeros], dim=1),
            torch.stack([s, c, zeros], dim=1),
            torch.stack([zeros, zeros, ones], dim=1),
        ],
        dim=1,
    )

    # Apply rotation: (N, 3, 3) @ (N, 3, 1) -> (N, 3, 1) -> (N, 3)
    world_predicted = (
        torch.bmm(R_z, robot_coords.unsqueeze(2)).squeeze(2) + robot_position
    )

    # Step 4: Target world positions for each observation's tag
    target_world = tag_world_positions[tag_idx]  # (N, 3)

    # Step 5: MSE loss
    diffs = world_predicted - target_world
    data_loss = torch.mean(torch.sum(diffs**2, dim=1))

    # Step 6: Regularization — penalize only the MEAN translation shift.
    # The gauge freedom is a global translation (shift all t_cam by the same
    # vector d, shift all tag positions to compensate). We break this 3-DOF
    # ambiguity by penalizing the mean shift, while allowing individual cameras
    # to adjust freely to find the true center of rotation.
    reg_weight = 10.0
    cam_ids = list(torch_params.keys())
    mean_shift = torch.zeros(3, dtype=torch.float32, device=device)
    for cam_id in cam_ids:
        mean_shift = mean_shift + (
            torch_params[cam_id]["translation"] - initial_translations[cam_id]
        )
    mean_shift = mean_shift / len(cam_ids)
    reg_loss = torch.sum(mean_shift**2)

    return data_loss + reg_weight * reg_loss


def run_optimizer(
    frameset_data,
    camera_params,
    device,
    num_iterations=2000,
    learning_rate=1e-2,
    lr_step_size=None,
    lr_gamma=0.5,
):
    """
    Run joint optimization of camera extrinsics, turntable angles, tag positions,
    and robot center.

    Args:
        frameset_data: Tuple of (observations, frame_ids, tag_ids).
        camera_params: Initial camera parameters dict.
        device: Torch device.
        num_iterations: Number of Adam iterations.
        learning_rate: Adam learning rate.

    Returns:
        Tuple of (torch_params, frame_angles, tag_world_positions, robot_position,
                  frame_ids, tag_ids).
    """
    observations, frame_ids, tag_ids = frameset_data

    torch_params = create_torch_params(camera_params, device)
    cam_ids = list(torch_params.keys())

    # Store initial translations (detached copies) for regularization
    initial_translations = {
        cam_id: torch_params[cam_id]["translation"].detach().clone()
        for cam_id in cam_ids
    }

    batch = preprocess_observations(observations, cam_ids, device)

    # Initialize optimizable parameters
    num_frames = batch["num_frames"]
    num_tags = batch["num_tags"]

    # Per-frame turntable angles (frame 0 fixed at 0)
    frame_angles_free = torch.zeros(
        num_frames - 1, dtype=torch.float32, device=device, requires_grad=True
    )

    # Per-tag world positions (initialized from first observations)
    init_tag_pos = initialize_tag_positions(batch, torch_params, device)
    tag_world_positions = init_tag_pos.clone().detach().requires_grad_(True)

    # Robot center position — fixed at origin to break the gauge ambiguity
    # between camera translations and robot position. Camera t_cam is defined
    # as the offset from robot center, so robot center IS the origin.
    robot_position = torch.zeros(
        3, dtype=torch.float32, device=device, requires_grad=False
    )

    # Collect all optimizable parameters
    opt_params = []
    for cam in torch_params.values():
        for p in cam.values():
            if p.requires_grad:
                opt_params.append(p)
    opt_params.append(frame_angles_free)
    opt_params.append(tag_world_positions)

    optimizer = optim.Adam(opt_params, lr=learning_rate)

    # Learning rate scheduling
    scheduler = None
    if lr_step_size is not None:
        scheduler = optim.lr_scheduler.StepLR(
            optimizer, step_size=lr_step_size, gamma=lr_gamma
        )
        print(f"LR schedule: StepLR(step_size={lr_step_size}, gamma={lr_gamma})")

    # Initial loss
    multiplier = 100.0
    loss = compute_loss(
        batch,
        torch_params,
        frame_angles_free,
        tag_world_positions,
        robot_position,
        initial_translations,
        device,
    )

    print("\n\n-------------------------------------------------")
    print(
        f"Initial Loss: {loss.item() * multiplier:.6f}, "
        f"RMSE: {loss.item()**0.5 * multiplier:.6f}"
    )
    print(
        f"Observations: {batch['num_obs']}, Frames: {num_frames}, " f"Tags: {num_tags}"
    )
    print("\nInitial Camera Parameters:")
    for camid, rec in torch_params.items():
        print(
            f"  {camid}: rot={rec['rotations'].tolist()}, "
            f"trans={rec['translation'].tolist()}"
        )
    print("Starting optimization...")
    print("-------------------------------------------------")

    # Optimization loop
    progress_bar = trange(num_iterations, desc="Optimizing")
    for _ in progress_bar:
        optimizer.zero_grad()
        loss = compute_loss(
            batch,
            torch_params,
            frame_angles_free,
            tag_world_positions,
            robot_position,
            initial_translations,
            device,
        )
        loss.backward()
        optimizer.step()
        if scheduler is not None:
            scheduler.step()
        current_lr = optimizer.param_groups[0]["lr"]
        progress_bar.set_postfix(
            loss=f"{loss.item() * multiplier:.6f}", lr=f"{current_lr:.1e}"
        )

    # Build full frame angles for output
    with torch.no_grad():
        zero = torch.zeros(1, dtype=torch.float32, device=device)
        frame_angles_full = torch.cat([zero, frame_angles_free])

    print("\n\n-------------------------------------------------")
    print(
        f"Final Loss: {loss.item() * multiplier:.6f}, "
        f"RMSE: {loss.item()**0.5 * multiplier:.6f}"
    )

    print_results(
        torch_params,
        frame_angles_full,
        tag_world_positions,
        robot_position,
        frame_ids,
        tag_ids,
    )

    print("-------------------------------------------------")

    return (
        torch_params,
        frame_angles_full,
        tag_world_positions,
        robot_position,
        frame_ids,
        tag_ids,
    )


def print_results(
    torch_params, frame_angles, tag_world_positions, robot_position, frame_ids, tag_ids
):
    """
    Print all optimized parameters in a readable format.

    Camera extrinsics are printed as JSON (same format as existing solver,
    copy-pasteable into system_config.json).
    """
    print("\n--- Camera Extrinsics ---")
    for camid, rec in torch_params.items():
        print(f"\nCamera ID: {camid}")
        print(f"  Rotations (deg): {rec['rotations'].detach().cpu().tolist()}")
        print(f"  Translation (m): {rec['translation'].detach().cpu().tolist()}")

        angles = rec["rotations"].detach().cpu().numpy()
        translation = rec["translation"].detach().cpu().numpy()
        R_cam = (
            compose_rotations_xyz_torch(angles[0], angles[1], angles[2])
            @ camera_to_robot_torch()
        )
        print(
            json.dumps(
                {
                    "rotation": R_cam.detach().cpu().numpy().tolist(),
                    "offset": translation.tolist(),
                },
                indent=4,
            )
        )

    print("\n--- Turntable Angles (degrees) ---")
    angles_np = frame_angles.detach().cpu().numpy()
    for i, fid in enumerate(frame_ids):
        fixed_str = " (fixed)" if i == 0 else ""
        print(f"  Frame {fid}: {angles_np[i]:.3f}{fixed_str}")

    print("\n--- Tag World Positions (meters) ---")
    tag_pos_np = tag_world_positions.detach().cpu().numpy()
    for i, tid in enumerate(tag_ids):
        pos = tag_pos_np[i]
        print(f"  Tag {tid}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

    robot_np = robot_position.detach().cpu().numpy()
    print(f"\n--- Robot Center Position (meters) ---")
    print(f"  [{robot_np[0]:.4f}, {robot_np[1]:.4f}, {robot_np[2]:.4f}]")


def write_diagnostic_config(
    config, torch_params, output_path="diagnose_config.yaml"
):
    """
    Write a YAML config file for diagnose_extrinsics.py using the solved parameters.

    Args:
        config: Original solver config dict (for frameset_dir and intrinsics paths).
        torch_params: Optimized camera parameter tensors.
        output_path: Path to write the diagnostic YAML.
    """
    diag = {
        "frameset_dir": config["frameset_dir"],
        "cameras": {},
    }

    for cam_id, rec in torch_params.items():
        angles = rec["rotations"].detach().cpu().numpy()
        translation = rec["translation"].detach().cpu().numpy()
        R_cam = (
            compose_rotations_xyz_torch(angles[0], angles[1], angles[2])
            @ camera_to_robot_torch()
        )
        rotation_list = R_cam.detach().cpu().numpy().tolist()
        offset_list = translation.tolist()

        diag["cameras"][cam_id] = {
            "intrinsics_filename": config["cameras"][cam_id]["intrinsics_filename"],
            "rotation": rotation_list,
            "offset": offset_list,
        }

    with open(output_path, "w") as f:
        yaml.dump(diag, f, default_flow_style=None, sort_keys=False)

    print(f"\nDiagnostic config written to: {output_path}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Solve for camera extrinsics using a rotating platform with AprilTags."
    )
    parser.add_argument("config", type=str, help="Path to configuration YAML file")
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Output path for diagnostic config YAML (default: auto-generated from camera IDs)",
    )
    parsed_args = parser.parse_args(args)

    config = read_yaml_file(parsed_args.config)
    device = get_device()

    camera_info, camera_params = setup_cameras(config)
    print("Camera Info:")
    for camid, rec in camera_info.items():
        print(f"  Camera {camid}:")
        print(f"    Intrinsic Matrix:\n{rec['intrinsic_matrix']}")
        print(f"    Distortion Coefficients: {rec['distortion_coeffs']}")

    frameset_data = generate_frameset_all_observations(config, camera_info)
    results = run_optimizer(
        frameset_data,
        camera_params,
        device,
        num_iterations=config["num_iterations"],
        learning_rate=config["learning_rate"],
        lr_step_size=config.get("lr_step_size"),
        lr_gamma=config.get("lr_gamma", 0.5),
    )

    torch_params = results[0]
    output_path = parsed_args.output
    if output_path is None:
        cam_ids = sorted(torch_params.keys())
        output_path = "_".join(cam_ids) + "_diagnostic_config.yaml"
    write_diagnostic_config(config, torch_params, output_path=output_path)


if __name__ == "__main__":
    main()
