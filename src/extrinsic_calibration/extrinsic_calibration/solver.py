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


def create_torch_params(params):
    """
    Convert camera parameters into torch tensors for optimization.

    Args:
        params (dict): Camera parameters.

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
            requires_grad=rotation_params_adjustable,
        )
        translation = torch.tensor(
            rec["translation"],
            dtype=torch.float32,
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


def compute_loss(frameset, torch_params):
    """
    Compute the loss based on the difference in detected AprilTag positions between pairs of cameras.

    The loss function calculates the mean squared error between the positions of the same AprilTags as viewed from two cameras,
    given the current estimates of camera parameters.

    Args:
        frameset (dict): Frameset data containing detected AprilTag positions.
        torch_params (dict): Current camera parameters as torch tensors.

    Returns:
        torch.Tensor: Computed mean squared error loss.
    """

    pair_diffs = []
    for fskey, fsvalue in frameset.items():
        for tagid, tagvalues in fsvalue.items():
            num_values = len(tagvalues)
            if num_values == 2:  # skip if we didnt see the tag in 2 cameras.
                pair = []
                for tagvalue in tagvalues:
                    cam_id = tagvalue["cam_id"]
                    at_camera_coords = torch.tensor(
                        tagvalue["translation"],
                        dtype=torch.float32,
                        requires_grad=False,
                    )
                    angles = torch_params[cam_id]["rotations"]
                    trans = torch_params[cam_id]["translation"]

                    R_cam = (
                        compose_rotations_xyz_torch(angles[0], angles[1], angles[2])
                        @ camera_to_robot_torch()
                    )
                    X_cam = R_cam @ at_camera_coords.T + trans
                    pair.append(X_cam)
                pair_diffs.append(pair[0] - pair[1])

    diffs_tensor = torch.stack(pair_diffs)
    loss = torch.mean(torch.sum(diffs_tensor**2, dim=1))  # <-- MSE loss
    return loss


def run_optimizer(frameset, params, num_iterations=500, learning_rate=1e-2):
    """
    Run optimization to refine camera parameters by minimizing differences in AprilTag positions between cameras.

    Uses Adam optimizer to iteratively adjust camera rotation and translation parameters, minimizing the computed loss.

    Args:
        frameset (dict): Frameset data containing detected AprilTag positions.
        params (dict): Initial camera parameters for optimization.
        num_iterations (int, optional): Number of optimization iterations. Defaults to 500.
        learning_rate (float, optional): Learning rate for optimizer. Defaults to 1e-2.

    Returns:
        None. Prints optimization progress and final optimized parameters.
    """

    torch_params = create_torch_params(params)

    optimizer = optim.Adam(
        [p for cam_params in torch_params.values() for p in cam_params.values()],
        lr=learning_rate,
    )

    # get a baseline loss before we start optimizing.
    multiplier = 100.0
    loss = compute_loss(frameset, torch_params)
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
        loss = compute_loss(frameset, torch_params)
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
        angles = rec["rotations"].detach().numpy()
        translation = rec["translation"].detach().numpy()
        R_cam = (
            compose_rotations_xyz_torch(angles[0], angles[1], angles[2])
            @ camera_to_robot_torch()
        )
        print(
            json.dumps(
                {
                    "rotation": R_cam.detach().numpy().tolist(),
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
        num_iterations=config["num_iterations"],
        learning_rate=config["learning_rate"],
    )


if __name__ == "__main__":
    main()
