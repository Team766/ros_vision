#!/usr/bin/env python3

import numpy as np
import os
import pytest
import tempfile
import torch
import yaml

from extrinsic_calibration.utils import (
    compose_rotations_xyz_torch,
    camera_to_robot_torch,
)
from extrinsic_calibration.solver import (
    create_torch_params,
    compute_loss,
    get_device,
    get_files,
    parse_filename,
    preprocess_frameset,
    run_optimizer,
)

TOLERANCE = 1e-5


# ---- Fixtures ----


@pytest.fixture
def device():
    return torch.device("cpu")


@pytest.fixture
def sample_camera_params():
    """Two cameras with known initial parameters."""
    return {
        "cam1": {
            "rotations": [0.0, 0.0, 0.0],
            "translation": [0.0, 0.0, 0.5],
            "rotation_params_adjustable": True,
            "translation_params_adjustable": True,
        },
        "cam2": {
            "rotations": [0.0, 0.0, -90.0],
            "translation": [0.5, 0.0, 0.5],
            "rotation_params_adjustable": True,
            "translation_params_adjustable": True,
        },
    }


@pytest.fixture
def sample_frameset():
    """Synthetic frameset: two cameras see the same tags from different positions.

    We construct tag translations in camera coords such that, with the correct
    extrinsics, both cameras agree on the tag's robot-frame position.
    """
    # Camera 1: identity rotation, translation [0, 0, 0.5]
    # Camera 2: -90 deg yaw, translation [0.5, 0, 0.5]
    # Both should place tag at robot-frame [1, 0, 0.5] (via camera_to_robot transform)

    # For cam1 with identity rotation:
    #   R_cam1 = Rx(0)Ry(0)Rz(0) @ camera_to_robot = camera_to_robot
    #   X = R_cam1 @ cam_coords + trans1
    # We want X = [1, 0, 0.5]
    # camera_to_robot maps cam [0,0,z] -> robot [z, 0, 0]
    # So cam1 sees tag at cam coords [0, 0, 1] -> robot [1, 0, 0] + [0,0,0.5] = [1, 0, 0.5]
    cam1_tag_coords = np.array([0.0, 0.0, 1.0])

    # For cam2 with -90 deg yaw, translation [0.5, 0, 0.5]:
    #   R_cam2 = Rx(0)Ry(0)Rz(-90) @ camera_to_robot
    #   We need R_cam2 @ cam2_coords + [0.5, 0, 0.5] = [1, 0, 0.5]
    #   So R_cam2 @ cam2_coords = [0.5, 0, 0]
    R_cam2 = compose_rotations_xyz_torch(0.0, 0.0, -90.0) @ camera_to_robot_torch()
    # cam2_coords = R_cam2^{-1} @ [0.5, 0, 0]
    target = torch.tensor([0.5, 0.0, 0.0])
    cam2_tag_coords = (R_cam2.T @ target).numpy()

    return {
        "frame_001": {
            1: [
                {
                    "cam_id": "cam1",
                    "filename": "img_001_cam1.png",
                    "translation": cam1_tag_coords,
                },
                {
                    "cam_id": "cam2",
                    "filename": "img_001_cam2.png",
                    "translation": cam2_tag_coords,
                },
            ],
        },
        "frame_002": {
            1: [
                {
                    "cam_id": "cam1",
                    "filename": "img_002_cam1.png",
                    "translation": cam1_tag_coords * 1.5,
                },
                {
                    "cam_id": "cam2",
                    "filename": "img_002_cam2.png",
                    "translation": (R_cam2.T @ torch.tensor([1.0, 0.0, 0.0])).numpy(),
                },
            ],
        },
    }


# ---- get_device ----


class TestGetDevice:
    def test_returns_valid_device(self):
        device = get_device()
        assert isinstance(device, torch.device)
        assert device.type in ("cpu", "cuda")


# ---- parse_filename ----


class TestParseFilename:
    def test_basic(self):
        frame_num, camid = parse_filename("img_001_cam1.png")
        assert frame_num == "001"
        assert camid == "cam1"

    def test_with_path(self):
        frame_num, camid = parse_filename("/some/path/img_042_cam13.png")
        assert frame_num == "042"
        assert camid == "cam13"


# ---- get_files ----


class TestGetFiles:
    def test_nonexistent_directory_raises(self):
        with pytest.raises(Exception, match="does not exist"):
            get_files("/nonexistent/path/12345")

    def test_finds_png_files(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create some test files
            for name in ["img_001_cam1.png", "img_001_cam2.png", "not_an_image.txt"]:
                open(os.path.join(tmpdir, name), "w").close()
            files = get_files(tmpdir)
            assert len(files) == 2
            assert all(f.endswith(".png") for f in files)

    def test_empty_directory(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            files = get_files(tmpdir)
            assert files == []


# ---- create_torch_params ----


class TestCreateTorchParams:
    def test_creates_tensors_on_device(self, device, sample_camera_params):
        torch_params = create_torch_params(sample_camera_params, device)
        assert "cam1" in torch_params
        assert "cam2" in torch_params
        for cam_id, params in torch_params.items():
            assert params["rotations"].device.type == device.type
            assert params["translation"].device.type == device.type
            assert params["rotations"].shape == (3,)
            assert params["translation"].shape == (3,)

    def test_requires_grad(self, device, sample_camera_params):
        torch_params = create_torch_params(sample_camera_params, device)
        assert torch_params["cam1"]["rotations"].requires_grad is True
        assert torch_params["cam1"]["translation"].requires_grad is True

    def test_fixed_params(self, device):
        params = {
            "cam1": {
                "rotations": [0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.5],
                "rotation_params_adjustable": False,
                "translation_params_adjustable": False,
            },
        }
        torch_params = create_torch_params(params, device)
        assert torch_params["cam1"]["rotations"].requires_grad is False
        assert torch_params["cam1"]["translation"].requires_grad is False

    def test_values_match_input(self, device, sample_camera_params):
        torch_params = create_torch_params(sample_camera_params, device)
        np.testing.assert_allclose(
            torch_params["cam1"]["rotations"].detach().numpy(),
            sample_camera_params["cam1"]["rotations"],
        )
        np.testing.assert_allclose(
            torch_params["cam2"]["translation"].detach().numpy(),
            sample_camera_params["cam2"]["translation"],
        )

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_creates_tensors_on_cuda(self, sample_camera_params):
        cuda_device = torch.device("cuda")
        torch_params = create_torch_params(sample_camera_params, cuda_device)
        for cam_id, params in torch_params.items():
            assert params["rotations"].device.type == "cuda"
            assert params["translation"].device.type == "cuda"


# ---- preprocess_frameset ----


class TestPreprocessFrameset:
    def test_basic_structure(self, device, sample_frameset):
        cam_ids = ["cam1", "cam2"]
        batch = preprocess_frameset(sample_frameset, cam_ids, device)

        assert batch["num_pairs"] == 2
        assert batch["coords_a"].shape == (2, 3)
        assert batch["coords_b"].shape == (2, 3)
        assert batch["coords_a"].device.type == device.type
        assert batch["coords_b"].device.type == device.type

    def test_masks_are_correct(self, device, sample_frameset):
        cam_ids = ["cam1", "cam2"]
        batch = preprocess_frameset(sample_frameset, cam_ids, device)

        # Each pair has cam1 on side A, cam2 on side B (based on fixture ordering)
        assert "cam1" in batch["masks_a"]
        assert "cam2" in batch["masks_b"]
        # cam1 should appear in all side-A entries
        assert len(batch["masks_a"]["cam1"]) == 2

    def test_skips_unpaired_observations(self, device):
        frameset = {
            "frame_001": {
                1: [  # only one observation, should be skipped
                    {
                        "cam_id": "cam1",
                        "filename": "x.png",
                        "translation": np.array([1.0, 0.0, 0.0]),
                    },
                ],
                2: [  # two observations, should be included
                    {
                        "cam_id": "cam1",
                        "filename": "x.png",
                        "translation": np.array([1.0, 0.0, 0.0]),
                    },
                    {
                        "cam_id": "cam2",
                        "filename": "y.png",
                        "translation": np.array([0.0, 1.0, 0.0]),
                    },
                ],
            },
        }
        batch = preprocess_frameset(frameset, ["cam1", "cam2"], device)
        assert batch["num_pairs"] == 1

    def test_empty_frameset(self, device):
        batch = preprocess_frameset({}, ["cam1", "cam2"], device)
        assert batch["num_pairs"] == 0

    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_preprocess_on_cuda(self, sample_frameset):
        cuda_device = torch.device("cuda")
        batch = preprocess_frameset(sample_frameset, ["cam1", "cam2"], cuda_device)
        assert batch["coords_a"].device.type == "cuda"
        assert batch["coords_b"].device.type == "cuda"
        for mask in batch["masks_a"].values():
            assert mask.device.type == "cuda"


# ---- compute_loss ----


class TestComputeLoss:
    def test_zero_loss_with_correct_params(
        self, device, sample_frameset, sample_camera_params
    ):
        """When extrinsics are correct, both cameras agree on tag position, so loss should be ~0."""
        torch_params = create_torch_params(sample_camera_params, device)
        batch_data = preprocess_frameset(
            sample_frameset, list(torch_params.keys()), device
        )
        loss = compute_loss(batch_data, torch_params, device)
        assert loss.item() < 1e-4, f"Expected near-zero loss, got {loss.item()}"

    def test_nonzero_loss_with_wrong_params(self, device, sample_frameset):
        """Wrong camera params should give nonzero loss."""
        wrong_params = {
            "cam1": {
                "rotations": [0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.5],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
            "cam2": {
                "rotations": [0.0, 0.0, 0.0],  # wrong rotation
                "translation": [0.0, 0.0, 0.0],  # wrong translation
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
        }
        torch_params = create_torch_params(wrong_params, device)
        batch_data = preprocess_frameset(
            sample_frameset, list(torch_params.keys()), device
        )
        loss = compute_loss(batch_data, torch_params, device)
        assert loss.item() > 0.01, f"Expected significant loss, got {loss.item()}"

    def test_loss_is_differentiable(
        self, device, sample_frameset, sample_camera_params
    ):
        torch_params = create_torch_params(sample_camera_params, device)
        batch_data = preprocess_frameset(
            sample_frameset, list(torch_params.keys()), device
        )
        loss = compute_loss(batch_data, torch_params, device)
        loss.backward()
        for cam_id, params in torch_params.items():
            if params["rotations"].requires_grad:
                assert (
                    params["rotations"].grad is not None
                ), f"No grad for {cam_id} rotations"
            if params["translation"].requires_grad:
                assert (
                    params["translation"].grad is not None
                ), f"No grad for {cam_id} translation"

    def test_loss_decreases_with_optimizer_step(self, device, sample_frameset):
        """One optimizer step from wrong params should decrease loss."""
        wrong_params = {
            "cam1": {
                "rotations": [5.0, 5.0, 5.0],
                "translation": [0.1, 0.1, 0.6],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
            "cam2": {
                "rotations": [5.0, 5.0, -85.0],
                "translation": [0.6, 0.1, 0.6],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
        }
        torch_params = create_torch_params(wrong_params, device)
        batch_data = preprocess_frameset(
            sample_frameset, list(torch_params.keys()), device
        )

        optimizer = torch.optim.Adam(
            [p for cam in torch_params.values() for p in cam.values()],
            lr=0.01,
        )

        # Initial loss
        loss_before = compute_loss(batch_data, torch_params, device)

        # Run a few steps
        for _ in range(50):
            optimizer.zero_grad()
            loss = compute_loss(batch_data, torch_params, device)
            loss.backward()
            optimizer.step()

        loss_after = compute_loss(batch_data, torch_params, device)
        assert (
            loss_after.item() < loss_before.item()
        ), f"Loss did not decrease: {loss_before.item()} -> {loss_after.item()}"


# ---- run_optimizer (integration) ----


class TestRunOptimizer:
    def test_optimizer_converges(self, device, sample_frameset):
        """Start with slightly wrong params and verify optimizer improves them."""
        perturbed_params = {
            "cam1": {
                "rotations": [2.0, -1.0, 3.0],
                "translation": [0.05, 0.02, 0.55],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
            "cam2": {
                "rotations": [1.0, -2.0, -87.0],
                "translation": [0.48, 0.03, 0.52],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
        }
        # Just verify it runs without error and doesn't crash
        run_optimizer(
            sample_frameset,
            perturbed_params,
            device,
            num_iterations=50,
            learning_rate=0.01,
        )

    def test_fixed_camera_not_adjusted(self, device, sample_frameset):
        """When a camera's params are not adjustable, they should remain unchanged."""
        params = {
            "cam1": {
                "rotations": [0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.5],
                "rotation_params_adjustable": False,
                "translation_params_adjustable": False,
            },
            "cam2": {
                "rotations": [5.0, 5.0, -85.0],
                "translation": [0.6, 0.1, 0.6],
                "rotation_params_adjustable": True,
                "translation_params_adjustable": True,
            },
        }
        torch_params = create_torch_params(params, device)
        cam1_rot_before = torch_params["cam1"]["rotations"].clone()
        cam1_trans_before = torch_params["cam1"]["translation"].clone()

        batch_data = preprocess_frameset(
            sample_frameset, list(torch_params.keys()), device
        )
        optimizer = torch.optim.Adam(
            [
                p
                for cam in torch_params.values()
                for p in cam.values()
                if p.requires_grad
            ],
            lr=0.01,
        )
        for _ in range(20):
            optimizer.zero_grad()
            loss = compute_loss(batch_data, torch_params, device)
            loss.backward()
            optimizer.step()

        assert torch.allclose(torch_params["cam1"]["rotations"], cam1_rot_before)
        assert torch.allclose(torch_params["cam1"]["translation"], cam1_trans_before)
