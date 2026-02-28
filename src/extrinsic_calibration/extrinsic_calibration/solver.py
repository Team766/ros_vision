#!/usr/bin/env python3

import argparse
import apriltag
import glob
import json
import numpy as np
import cv2
import torch
import torch.optim as optim
from tqdm import trange
import os

# Try to import from local utils, fallback to standalone utils
try:
    from .utils import (
        read_yaml_file,
        compose_rotations_xyz_torch,
        camera_to_robot_torch,
    )
except ImportError:
    # Standalone mode - import from same directory
    from utils import read_yaml_file, compose_rotations_xyz_torch, camera_to_robot_torch


def get_device():
    """Select CUDA device if available, otherwise CPU."""
    if torch.cuda.is_available():
        device = torch.device("cuda")
        print(f"Using CUDA device: {torch.cuda.get_device_name(0)}")
    else:
        device = torch.device("cpu")
        print("CUDA not available, using CPU")
    return device


def get_files(dir_name):
    """
    Retrieve all PNG image file paths from the specified directory.

    Args:
        dir_name (str): Directory path to search for PNG files.

    Returns:
        list: List of file paths matching the PNG file pattern.
    """

    import os

    if not os.path.exists(dir_name):
        raise Exception(f"Error: directory does not exist {dir_name}")

    files = glob.glob(os.path.join(dir_name, "*.png"))
    return files


def parse_filename(filename):
    """
    Parse the filename to extract the frame number and camera ID.

    Args:
        filename (str): Filename to parse.

    Returns:
        tuple: Frame number and camera ID extracted from the filename.
    """
    base, ext = os.path.splitext(os.path.basename(filename))
    _, frame_num, camid = base.split("_")
    return frame_num, camid


def draw_tag(img, det):
    """
    Draw the detected AprilTag onto the image.

    Args:
        img (numpy.ndarray): The image on which to draw.
        det (apriltag.Detection): The AprilTag detection object.
    """
    color = (0, 0, 255)
    thickness = 2
    cv2.line(
        img,
        (det.corners[0, 0].astype(np.int32), det.corners[0, 1].astype(np.int32)),
        (det.corners[1, 0].astype(np.int32), det.corners[1, 1].astype(np.int32)),
        color,
        thickness,
    )
    cv2.line(
        img,
        (det.corners[1, 0].astype(np.int32), det.corners[1, 1].astype(np.int32)),
        (det.corners[2, 0].astype(np.int32), det.corners[2, 1].astype(np.int32)),
        color,
        thickness,
    )
    cv2.line(
        img,
        (det.corners[2, 0].astype(np.int32), det.corners[2, 1].astype(np.int32)),
        (det.corners[3, 0].astype(np.int32), det.corners[3, 1].astype(np.int32)),
        color,
        thickness,
    )
    cv2.line(
        img,
        (det.corners[3, 0].astype(np.int32), det.corners[3, 1].astype(np.int32)),
        (det.corners[0, 0].astype(np.int32), det.corners[0, 1].astype(np.int32)),
        color,
        thickness,
    )

    label = f"ID: {det.tag_id}"
    (text_width, text_height), _ = cv2.getTextSize(
        label, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, 2
    )

    # Compute the text position, cast to int because OpenCV needs ints
    text_x = int(det.center[0] - text_width / 2)
    text_y = int(det.center[1] + text_height / 2)
    cv2.putText(
        img,
        label,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
        1.0,
        (255, 0, 0),
        2,
    )


def setup_cameras(config):
    """
    Load camera intrinsic parameters, distortion coefficients, and initialize camera rotations and translations.

    Args:
        config (dict): Configuration dictionary containing camera settings and initial parameters.

    Returns:
        tuple:
            - camera_info (dict): Camera information with intrinsic matrices and distortion coefficients.
            - camera_params (dict): Initialized camera rotations and translations ready for optimization.
    """
    camera_info = config["cameras"]
    camera_params = {}

    for camid, rec in camera_info.items():
        intrinsic_data = read_yaml_file(rec["intrinsics_filename"])
        rec["intrinsic_matrix"] = np.array(intrinsic_data["matrix"], dtype=np.float32)
        rec["distortion_coeffs"] = np.array(intrinsic_data["disto"]).squeeze(0)

        camera_params[camid] = {
            "rotations": rec["initial_rotations_degrees"],
            "translation": rec["initial_translation_meters"],
            "rotation_params_adjustable": rec["rotation_params_adjustable"],
            "translation_params_adjustable": rec["translation_params_adjustable"],
        }

    return camera_info, camera_params


def create_torch_params(params, device):
    """
    Convert camera parameters into torch tensors for optimization.

    Args:
        params (dict): Camera parameters.
        device (torch.device): Device to create tensors on.

    Returns:
        dict: Torch tensors for camera rotations and translations.
    """
    torch_params = {}
    for camid, rec in params.items():
        rotation_params_adjustable = rec["rotation_params_adjustable"]
        translation_params_adjustable = rec["translation_params_adjustable"]
        rotations = torch.tensor(
            rec["rotations"],
            dtype=torch.float32,
            device=device,
            requires_grad=rotation_params_adjustable,
        )
        translation = torch.tensor(
            rec["translation"],
            dtype=torch.float32,
            device=device,
            requires_grad=translation_params_adjustable,
        )
        torch_params[camid] = {"rotations": rotations, "translation": translation}
    return torch_params


def generate_frameset(config, cams_dict):
    """
    Generate a dataset (frameset) by detecting AprilTags in images from multiple cameras.

    Args:
        config (dict): Configuration containing frameset directory and detection settings.
        cams_dict (dict): Dictionary containing intrinsic parameters for each camera.

    Returns:
        dict: Nested dictionary containing detected tag positions organized by frame number and tag ID.
    """

    frame_set = {}

    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    show = False

    files = get_files(config["frameset_dir"])
    files.sort()
    for filename in files:
        frame = cv2.imread(filename)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection_list = detector.detect(grey)
        frame_num, camid = parse_filename(filename)
        if frame_num not in frame_set.keys():
            frame_set[frame_num] = {}

        if len(detection_list) > 0:
            for det in detection_list:
                id = det.tag_id
                m = cams_dict[camid]["intrinsic_matrix"]
                params = (m[0, 0], m[1, 1], m[0, 2], m[1, 2])
                solvedpose = detector.detection_pose(det, params, tag_size=0.1651)
                translation = solvedpose[0][:3, 3]
                rec = {
                    "cam_id": camid,
                    "filename": os.path.basename(filename),
                    "translation": translation,
                }
                if id not in frame_set[frame_num].keys():
                    frame_set[frame_num][id] = [rec]
                else:
                    frame_set[frame_num][id].append(rec)
                draw_tag(frame, det)
        if show:
            cv2.imshow("Apriltag Viewer", frame)
            _ = cv2.waitKey(0)

    return frame_set


def preprocess_frameset(frameset, cam_ids, device):
    """
    Pre-batch frameset observations into tensors for efficient GPU computation.

    Extracts all tag observation pairs and organizes them into batched tensors
    with pre-computed per-camera index masks, so compute_loss does no Python-level
    looping or tensor allocation.

    Args:
        frameset (dict): Frameset data from generate_frameset.
        cam_ids (list): List of camera IDs (keys of torch_params).
        device (torch.device): Device to create tensors on.

    Returns:
        dict: Batched data with coords tensors and per-camera index masks.
    """
    all_coords_a = []
    all_coords_b = []
    all_cam_ids_a = []
    all_cam_ids_b = []

    for fsvalue in frameset.values():
        for tagvalues in fsvalue.values():
            if len(tagvalues) == 2:
                all_coords_a.append(tagvalues[0]["translation"])
                all_cam_ids_a.append(tagvalues[0]["cam_id"])
                all_coords_b.append(tagvalues[1]["translation"])
                all_cam_ids_b.append(tagvalues[1]["cam_id"])

    num_pairs = len(all_coords_a)
    print(f"Preprocessed {num_pairs} observation pairs for optimization")

    # Stack all coords into (N, 3) tensors on device once
    coords_a = torch.tensor(np.array(all_coords_a), dtype=torch.float32, device=device)
    coords_b = torch.tensor(np.array(all_coords_b), dtype=torch.float32, device=device)

    # Pre-compute per-camera index masks as tensors
    masks_a = {}
    masks_b = {}
    for cam_id in cam_ids:
        idx_a = [i for i, c in enumerate(all_cam_ids_a) if c == cam_id]
        idx_b = [i for i, c in enumerate(all_cam_ids_b) if c == cam_id]
        if idx_a:
            masks_a[cam_id] = torch.tensor(idx_a, dtype=torch.long, device=device)
        if idx_b:
            masks_b[cam_id] = torch.tensor(idx_b, dtype=torch.long, device=device)

    return {
        "coords_a": coords_a,
        "coords_b": coords_b,
        "masks_a": masks_a,
        "masks_b": masks_b,
        "cam_ids": cam_ids,
        "num_pairs": num_pairs,
    }


def compute_loss(batch_data, torch_params, device):
    """
    Compute the MSE loss using pre-batched observation data.

    For each camera, applies the rotation and translation to all observations
    from that camera in a single batched matmul, then computes pairwise differences.

    Args:
        batch_data (dict): Pre-batched data from preprocess_frameset.
        torch_params (dict): Current camera parameters as torch tensors.
        device (torch.device): Device tensors live on.

    Returns:
        torch.Tensor: Computed mean squared error loss.
    """
    coords_a = batch_data["coords_a"]  # (N, 3)
    coords_b = batch_data["coords_b"]  # (N, 3)

    X_a = torch.empty_like(coords_a)
    X_b = torch.empty_like(coords_b)

    for cam_id in batch_data["cam_ids"]:
        angles = torch_params[cam_id]["rotations"]
        trans = torch_params[cam_id]["translation"]
        R_cam = compose_rotations_xyz_torch(
            angles[0], angles[1], angles[2]
        ) @ camera_to_robot_torch(device=device)

        # Side A: batched (3,3) @ (3,M) -> (3,M), then transpose back to (M,3)
        if cam_id in batch_data["masks_a"]:
            idx = batch_data["masks_a"][cam_id]
            c = coords_a[idx]  # (M, 3)
            X_a[idx] = (R_cam @ c.mT).mT + trans

        # Side B
        if cam_id in batch_data["masks_b"]:
            idx = batch_data["masks_b"][cam_id]
            c = coords_b[idx]  # (M, 3)
            X_b[idx] = (R_cam @ c.mT).mT + trans

    diffs = X_a - X_b
    loss = torch.mean(torch.sum(diffs**2, dim=1))  # MSE loss
    return loss


def run_optimizer(frameset, params, device, num_iterations=500, learning_rate=1e-2):
    """
    Run optimization to refine camera parameters by minimizing differences in AprilTag positions between cameras.

    Uses Adam optimizer to iteratively adjust camera rotation and translation parameters, minimizing the computed loss.

    Args:
        frameset (dict): Frameset data containing detected AprilTag positions.
        params (dict): Initial camera parameters for optimization.
        device (torch.device): Device to run optimization on.
        num_iterations (int, optional): Number of optimization iterations. Defaults to 500.
        learning_rate (float, optional): Learning rate for optimizer. Defaults to 1e-2.

    Returns:
        None. Prints optimization progress and final optimized parameters.
    """

    torch_params = create_torch_params(params, device)

    # Pre-batch all observation data into GPU tensors once
    batch_data = preprocess_frameset(frameset, list(torch_params.keys()), device)

    optimizer = optim.Adam(
        [p for cam_params in torch_params.values() for p in cam_params.values()],
        lr=learning_rate,
    )

    # get a baseline loss before we start optimizing.
    multiplier = 100.0
    loss = compute_loss(batch_data, torch_params, device)
    print("\n\n-------------------------------------------------")
    print(
        f"Initial Loss: {loss.item() * multiplier:.6f}, RMSE: {loss.item()**0.5 * multiplier:.6f}"
    )
    print("Initial Parameters:")
    for camid, rec in torch_params.items():
        print(f"Camera ID: {camid}")
        print(f"Rotations: {rec['rotations']}")
        print(f"Translation: {rec['translation']}")
    print("Starting optimization...")
    print("-------------------------------------------------")

    progress_bar = trange(num_iterations, desc="Optimizing")
    for it in progress_bar:
        optimizer.zero_grad()
        loss = compute_loss(batch_data, torch_params, device)
        loss.backward()
        optimizer.step()

        progress_bar.set_postfix(loss=f"{loss.item() * multiplier:.6f}")

    print("\n\n-------------------------------------------------")
    print("Final Parameters:")
    print(
        f"Final Loss: {loss.item() * multiplier:.6f}, RMSE: {loss.item()**0.5 * multiplier:.6f}"
    )
    print_updated_params(torch_params)
    print("-----------------------------------------------------")


def print_updated_params(torch_params):
    """
    Print updated camera parameters after optimization.

    Args:
        params (dict): Updated camera parameters.

    Returns:
        None. Prints the updated parameters.
    """

    for camid, rec in torch_params.items():
        print(f"Camera ID: {camid}")
        print(f"Rotations: {rec['rotations']}")
        print(f"Translation: {rec['translation']}")
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
        print("\n\n")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Solve for camera extrinsics using AprilTags."
    )
    parser.add_argument("config", type=str, help="Path to configuration file")
    parsed_args = parser.parse_args(args)

    config = read_yaml_file(parsed_args.config)
    device = get_device()

    camera_info, camera_params = setup_cameras(config)
    print("Camera Info:")
    for camid, rec in camera_info.items():
        print(f"Camera ID: {camid}")
        print(f"Intrinsic Matrix:\n{rec['intrinsic_matrix']}")
        print(f"Distortion Coefficients:\n{rec['distortion_coeffs']}")

    frameset = generate_frameset(config, camera_info)
    run_optimizer(
        frameset,
        camera_params,
        device,
        num_iterations=config["num_iterations"],
        learning_rate=config["learning_rate"],
    )


if __name__ == "__main__":
    main()
