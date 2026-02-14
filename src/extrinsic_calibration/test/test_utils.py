#!/usr/bin/env python3

import numpy as np
import pytest
import torch

from extrinsic_calibration.utils import (
    camera_to_robot_torch,
    compose_rotations_xyz_torch,
    rotation_x_torch,
    rotation_y_torch,
    rotation_z_torch,
)

TOLERANCE = 1e-5


# ---- Helpers ----


def assert_orthogonal(R, tol=TOLERANCE):
    """Verify R is a valid rotation matrix: R^T @ R == I, det(R) == +1."""
    eye = torch.eye(3)
    assert torch.allclose(R.T @ R, eye, atol=tol), f"R^T @ R != I:\n{R.T @ R}"
    assert torch.isclose(
        torch.det(R), torch.tensor(1.0), atol=tol
    ), f"det(R) = {torch.det(R).item()}, expected 1.0"


# ---- rotation_x_torch ----


class TestRotationX:
    def test_zero_degrees_is_identity(self):
        R = rotation_x_torch(0.0)
        assert torch.allclose(R, torch.eye(3), atol=TOLERANCE)

    def test_90_degrees(self):
        R = rotation_x_torch(90.0)
        # Rx(90): y -> z, z -> -y
        v = torch.tensor([0.0, 1.0, 0.0])
        result = R @ v
        expected = torch.tensor([0.0, 0.0, 1.0])
        assert torch.allclose(result, expected, atol=TOLERANCE)

    def test_180_degrees(self):
        R = rotation_x_torch(180.0)
        v = torch.tensor([0.0, 1.0, 0.0])
        result = R @ v
        expected = torch.tensor([0.0, -1.0, 0.0])
        assert torch.allclose(result, expected, atol=TOLERANCE)

    def test_orthogonality(self):
        for angle in [0.0, 30.0, 45.0, 90.0, 135.0, 180.0, -45.0, 270.0]:
            R = rotation_x_torch(angle)
            assert_orthogonal(R)

    def test_x_axis_unchanged(self):
        R = rotation_x_torch(47.0)
        v = torch.tensor([1.0, 0.0, 0.0])
        result = R @ v
        assert torch.allclose(result, v, atol=TOLERANCE)

    def test_tensor_input(self):
        angle = torch.tensor(90.0, requires_grad=True)
        R = rotation_x_torch(angle)
        assert_orthogonal(R)
        # Verify gradient flows
        loss = R.sum()
        loss.backward()
        assert angle.grad is not None

    def test_inverse(self):
        R_pos = rotation_x_torch(30.0)
        R_neg = rotation_x_torch(-30.0)
        assert torch.allclose(R_pos @ R_neg, torch.eye(3), atol=TOLERANCE)


# ---- rotation_y_torch ----


class TestRotationY:
    def test_zero_degrees_is_identity(self):
        R = rotation_y_torch(0.0)
        assert torch.allclose(R, torch.eye(3), atol=TOLERANCE)

    def test_90_degrees(self):
        R = rotation_y_torch(90.0)
        # Ry(90): z -> x, x -> -z
        v = torch.tensor([0.0, 0.0, 1.0])
        result = R @ v
        expected = torch.tensor([1.0, 0.0, 0.0])
        assert torch.allclose(result, expected, atol=TOLERANCE)

    def test_orthogonality(self):
        for angle in [0.0, 30.0, 45.0, 90.0, -90.0, 180.0]:
            R = rotation_y_torch(angle)
            assert_orthogonal(R)

    def test_y_axis_unchanged(self):
        R = rotation_y_torch(63.0)
        v = torch.tensor([0.0, 1.0, 0.0])
        result = R @ v
        assert torch.allclose(result, v, atol=TOLERANCE)

    def test_inverse(self):
        R_pos = rotation_y_torch(45.0)
        R_neg = rotation_y_torch(-45.0)
        assert torch.allclose(R_pos @ R_neg, torch.eye(3), atol=TOLERANCE)


# ---- rotation_z_torch ----


class TestRotationZ:
    def test_zero_degrees_is_identity(self):
        R = rotation_z_torch(0.0)
        assert torch.allclose(R, torch.eye(3), atol=TOLERANCE)

    def test_90_degrees(self):
        R = rotation_z_torch(90.0)
        # Rz(90): x -> y, y -> -x
        v = torch.tensor([1.0, 0.0, 0.0])
        result = R @ v
        expected = torch.tensor([0.0, 1.0, 0.0])
        assert torch.allclose(result, expected, atol=TOLERANCE)

    def test_orthogonality(self):
        for angle in [0.0, 30.0, 45.0, 90.0, -90.0, 180.0]:
            R = rotation_z_torch(angle)
            assert_orthogonal(R)

    def test_z_axis_unchanged(self):
        R = rotation_z_torch(120.0)
        v = torch.tensor([0.0, 0.0, 1.0])
        result = R @ v
        assert torch.allclose(result, v, atol=TOLERANCE)

    def test_inverse(self):
        R_pos = rotation_z_torch(60.0)
        R_neg = rotation_z_torch(-60.0)
        assert torch.allclose(R_pos @ R_neg, torch.eye(3), atol=TOLERANCE)


# ---- compose_rotations_xyz_torch ----


class TestComposeRotations:
    def test_all_zeros_is_identity(self):
        R = compose_rotations_xyz_torch(0.0, 0.0, 0.0)
        assert torch.allclose(R, torch.eye(3), atol=TOLERANCE)

    def test_matches_individual(self):
        roll, pitch, yaw = 10.0, 20.0, 30.0
        R_composed = compose_rotations_xyz_torch(roll, pitch, yaw)
        R_manual = (
            rotation_x_torch(roll) @ rotation_y_torch(pitch) @ rotation_z_torch(yaw)
        )
        assert torch.allclose(R_composed, R_manual, atol=TOLERANCE)

    def test_orthogonality(self):
        R = compose_rotations_xyz_torch(15.0, -25.0, 45.0)
        assert_orthogonal(R)

    def test_gradients_flow(self):
        roll = torch.tensor(10.0, requires_grad=True)
        pitch = torch.tensor(20.0, requires_grad=True)
        yaw = torch.tensor(30.0, requires_grad=True)
        R = compose_rotations_xyz_torch(roll, pitch, yaw)
        loss = R.sum()
        loss.backward()
        assert roll.grad is not None
        assert pitch.grad is not None
        assert yaw.grad is not None


# ---- camera_to_robot_torch ----


class TestCameraToRobot:
    def test_is_valid_rotation(self):
        R = camera_to_robot_torch()
        assert_orthogonal(R)

    def test_expected_transform(self):
        """camera_to_robot is compose_rotations_xyz(-90, 90, 0).
        Camera convention: z=forward, x=right, y=down.
        Robot convention: x=forward, y=left, z=up.
        Camera z-axis (forward) should map to robot x-axis (forward)."""
        R = camera_to_robot_torch()
        cam_forward = torch.tensor([0.0, 0.0, 1.0])
        robot_result = R @ cam_forward
        # Camera z -> robot x
        expected = torch.tensor([1.0, 0.0, 0.0])
        assert torch.allclose(robot_result, expected, atol=TOLERANCE)

    def test_matches_explicit_composition(self):
        R = camera_to_robot_torch()
        R_explicit = compose_rotations_xyz_torch(-90.0, 90.0, 0.0)
        assert torch.allclose(R, R_explicit, atol=TOLERANCE)


# ---- Device tests ----


class TestDeviceSupport:
    def test_rotation_on_cpu_explicit(self):
        device = torch.device("cpu")
        R = rotation_x_torch(45.0, device=device)
        assert R.device.type == "cpu"
        assert_orthogonal(R)

    def test_compose_on_cpu_explicit(self):
        device = torch.device("cpu")
        R = compose_rotations_xyz_torch(10.0, 20.0, 30.0, device=device)
        assert R.device.type == "cpu"
        assert_orthogonal(R)

    def test_camera_to_robot_on_cpu_explicit(self):
        device = torch.device("cpu")
        R = camera_to_robot_torch(device=device)
        assert R.device.type == "cpu"

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_rotation_on_cuda(self):
        device = torch.device("cuda")
        R = rotation_x_torch(45.0, device=device)
        assert R.device.type == "cuda"
        assert_orthogonal(R.cpu())

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_compose_on_cuda(self):
        device = torch.device("cuda")
        R = compose_rotations_xyz_torch(10.0, 20.0, 30.0, device=device)
        assert R.device.type == "cuda"
        assert_orthogonal(R.cpu())

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_camera_to_robot_on_cuda(self):
        device = torch.device("cuda")
        R = camera_to_robot_torch(device=device)
        assert R.device.type == "cuda"

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_tensor_input_on_cuda(self):
        """When angle is a CUDA tensor, result should stay on CUDA without explicit device."""
        angle = torch.tensor(45.0, device="cuda", requires_grad=True)
        R = rotation_x_torch(angle)
        assert R.device.type == "cuda"
        loss = R.sum()
        loss.backward()
        assert angle.grad is not None
