#!/usr/bin/env python3
"""Convert YOLO PyTorch weights to ONNX format."""

import argparse
from pathlib import Path
from typing import Optional

from ultralytics import YOLO


def convert_to_onnx(
    weights_path: str,
    output_path: Optional[str] = None,
    imgsz: int = 640,
    opset: int = 12,
    simplify: bool = True,
    dynamic: bool = False,
    half: bool = False,
) -> Path:
    """
    Convert YOLO PyTorch weights to ONNX format.

    Args:
        weights_path: Path to the PyTorch weights file (.pt)
        output_path: Optional output path for ONNX file. If None, uses same name as input.
        imgsz: Input image size (default: 640)
        opset: ONNX opset version (default: 12)
        simplify: Whether to simplify the ONNX model (default: True)
        dynamic: Whether to use dynamic input shapes (default: False)
        half: Whether to export in FP16 (default: False)

    Returns:
        Path to the exported ONNX file
    """
    model = YOLO(weights_path)

    export_args = {
        "format": "onnx",
        "imgsz": imgsz,
        "opset": opset,
        "simplify": simplify,
        "dynamic": dynamic,
        "half": half,
    }

    result = model.export(**export_args)

    if output_path and result != output_path:
        result_path = Path(result)
        output_path = Path(output_path)
        result_path.rename(output_path)
        return output_path

    return Path(result)


def main():
    parser = argparse.ArgumentParser(
        description="Convert YOLO PyTorch weights to ONNX format"
    )
    parser.add_argument(
        "weights",
        type=str,
        help="Path to PyTorch weights file (.pt)",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Output path for ONNX file (default: same as input with .onnx extension)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Input image size (default: 640)",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=12,
        help="ONNX opset version (default: 12)",
    )
    parser.add_argument(
        "--no-simplify",
        action="store_true",
        help="Disable ONNX model simplification",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Enable dynamic input shapes",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Export in FP16 half precision",
    )

    args = parser.parse_args()

    output_path = convert_to_onnx(
        weights_path=args.weights,
        output_path=args.output,
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=not args.no_simplify,
        dynamic=args.dynamic,
        half=args.half,
    )

    print(f"ONNX model exported to: {output_path}")


if __name__ == "__main__":
    main()
