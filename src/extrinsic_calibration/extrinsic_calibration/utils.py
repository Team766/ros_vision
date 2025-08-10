#!/usr/bin/env python3

import numpy as np
import torch
import yaml
from typing import Dict, Any


def read_yaml_file(file_path: str) -> Dict[str, Any]:
    """
    Reads a YAML file and returns the parsed content.

    Parameters:
        file_path (str): Path to the YAML file.

    Returns:
        dict: Parsed YAML data.
    """
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def rotation_x_torch(angle_degrees):
    """
    Returns a 3x3 rotation matrix (as a torch tensor) for a rotation about the X-axis.
    angle_degrees can be a torch scalar (with grad) or a float.
    """
    if torch.is_tensor(angle_degrees):
        theta = angle_degrees * (torch.pi / 180.0)
    else:
        theta = torch.tensor(
            angle_degrees * (np.pi / 180.0),
            dtype=torch.float32,
            device="cpu",
            requires_grad=False,
        )
    c = torch.cos(theta)
    s = torch.sin(theta)
    one = torch.tensor(1.0, dtype=torch.float32, device=theta.device)
    zero = torch.tensor(0.0, dtype=torch.float32, device=theta.device)
    return torch.stack(
        [
            torch.stack([one, zero, zero]),
            torch.stack([zero, c, -s]),
            torch.stack([zero, s, c]),
        ]
    )


def rotation_y_torch(angle_degrees):
    """
    Returns a 3x3 rotation matrix (as a torch tensor) for a rotation about the Y-axis.
    """
    if torch.is_tensor(angle_degrees):
        theta = angle_degrees * (torch.pi / 180.0)
    else:
        theta = torch.tensor(
            angle_degrees * (np.pi / 180.0),
            dtype=torch.float32,
            device="cpu",
            requires_grad=False,
        )
    c = torch.cos(theta)
    s = torch.sin(theta)
    one = torch.tensor(1.0, dtype=torch.float32, device=theta.device)
    zero = torch.tensor(0.0, dtype=torch.float32, device=theta.device)
    return torch.stack(
        [
            torch.stack([c, zero, s]),
            torch.stack([zero, one, zero]),
            torch.stack([-s, zero, c]),
        ]
    )


def rotation_z_torch(angle_degrees):
    """
    Returns a 3x3 rotation matrix (as a torch tensor) for a rotation about the Z-axis.
    """
    if torch.is_tensor(angle_degrees):
        theta = angle_degrees * (torch.pi / 180.0)
    else:
        theta = torch.tensor(
            angle_degrees * (np.pi / 180.0),
            dtype=torch.float32,
            device="cpu",
            requires_grad=False,
        )
    c = torch.cos(theta)
    s = torch.sin(theta)
    one = torch.tensor(1.0, dtype=torch.float32, device=theta.device)
    zero = torch.tensor(0.0, dtype=torch.float32, device=theta.device)
    return torch.stack(
        [
            torch.stack([c, -s, zero]),
            torch.stack([s, c, zero]),
            torch.stack([zero, zero, one]),
        ]
    )


def compose_rotations_xyz_torch(roll_deg, pitch_deg, yaw_deg):
    """
    Compose rotations in the order Rx(roll) * Ry(pitch) * Rz(yaw).
    """
    Rx = rotation_x_torch(roll_deg)
    Ry = rotation_y_torch(pitch_deg)
    Rz = rotation_z_torch(yaw_deg)
    return Rx @ Ry @ Rz


def camera_to_robot_torch():
    """
    Converts camera coordinates to robot frame.
    """
    return compose_rotations_xyz_torch(-90.0, 90.0, 0.0)