#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import json
import os
import time


class CharucoCalibrationNode2(Node):
    def __init__(self):
        super().__init__(
            'charuco_calibrator_node',
            allow_undeclared_parameters=True,
        )

        # Declare parameters
        self.declare_parameter('camera_serial', '')
        self.declare_parameter('board_cols', 11)
        self.declare_parameter('board_rows', 8)
        self.declare_parameter('square_length', 0.060)
        self.declare_parameter('marker_length', 0.045)
        self.declare_parameter('max_frames', 30)
        self.declare_parameter('frame_rate_hz', 2.0)
        self.declare_parameter('publisher_topic',
                               '/calibration/image_annotated/compressed')

        camera_serial = self.get_parameter('camera_serial').value
        if not camera_serial:
            self.get_logger().error(
                "camera_serial parameter is required. "
                "Usage: ros2 run camera_calibration charuco_calibrator2 "
                "--ros-args -p camera_serial:=<serial>")
            raise RuntimeError("camera_serial parameter is required")

        self.board_cols = self.get_parameter('board_cols').value
        self.board_rows = self.get_parameter('board_rows').value
        self.square_length = self.get_parameter('square_length').value
        self.marker_length = self.get_parameter('marker_length').value
        self.max_frames = self.get_parameter('max_frames').value
        self.frame_rate_hz = self.get_parameter('frame_rate_hz').value
        publisher_topic = self.get_parameter('publisher_topic').value

        self.bridge = CvBridge()

        # Set up charuco board and detector
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.board = aruco.CharucoBoard(
            (self.board_cols, self.board_rows),
            squareLength=self.square_length,
            markerLength=self.marker_length,
            dictionary=self.dictionary
        )
        self.detector = aruco.CharucoDetector(self.board)

        # Publisher for annotated images
        self.publisher = self.create_publisher(
            CompressedImage, publisher_topic, 10)

        # Open the camera directly
        cap = self._open_camera(camera_serial)
        if cap is None:
            raise RuntimeError(
                f"Failed to open camera with serial {camera_serial}")

        # Run calibration
        self._calibrate_camera(cap, camera_serial)
        cap.release()
        self.get_logger().info("Calibration complete. Press ctrl-c to exit.")

    def _open_camera(self, camera_serial):
        """Find and open the camera matching the given serial number."""
        from ros_vision_launch.utils import scan_for_cameras

        cameras_by_serial = scan_for_cameras()
        if camera_serial not in cameras_by_serial:
            self.get_logger().error(
                f"Camera serial '{camera_serial}' not found. "
                f"Available: {list(cameras_by_serial.keys())}")
            return None

        video_index = cameras_by_serial[camera_serial]

        # Load camera config from system_config.json
        config = self._load_camera_config(camera_serial)

        width = config.get('width', 1280)
        height = config.get('height', 800)

        api_preference = cv2.CAP_ANY
        if 'api_preference' in config:
            api_map = {
                'V4L2': cv2.CAP_V4L2,
                'GSTREAMER': cv2.CAP_GSTREAMER,
                'FFMPEG': cv2.CAP_FFMPEG,
                'ANY': cv2.CAP_ANY,
            }
            api_preference = api_map.get(
                config['api_preference'], cv2.CAP_ANY)

        self.get_logger().info(
            f"Opening camera {camera_serial} at video index "
            f"{video_index} ({width}x{height})...")

        cap = cv2.VideoCapture(video_index, api_preference)
        if not cap.isOpened():
            self.get_logger().error(
                f"Cannot open camera at video index {video_index}")
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if 'format' in config:
            fourcc = cv2.VideoWriter_fourcc(*config['format'])
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        if 'frame_rate' in config:
            cap.set(cv2.CAP_PROP_FPS, config['frame_rate'])

        self.get_logger().info(f"Camera {camera_serial} opened successfully.")
        return cap

    def _load_camera_config(self, camera_serial):
        """Load config for a single camera from system_config.json."""
        from ament_index_python.packages import get_package_share_directory

        data_path = os.path.join(
            get_package_share_directory('vision_config_data'),
            'data', 'system_config.json')

        with open(data_path, 'r') as f:
            config_data = json.load(f)

        positions = config_data['camera_mounted_positions']
        if camera_serial not in positions:
            raise RuntimeError(
                f"Camera serial '{camera_serial}' not found in "
                f"system_config.json camera_mounted_positions")

        return positions[camera_serial]

    def _calibrate_camera(self, cap, camera_serial):
        """Run the charuco calibration capture loop."""
        self.get_logger().info(
            f"Starting calibration for camera {camera_serial} "
            f"at {self.frame_rate_hz} Hz. "
            f"Collecting {self.max_frames} frames...")

        all_corners = []
        all_ids = []
        captured_frames = 0
        image_size = None

        while captured_frames < self.max_frames:
            frame_start = time.perf_counter()

            ret, frame = cap.read()
            if not ret:
                self.get_logger().warning("Failed to read frame, retrying...")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = gray.shape[::-1]

            corners, ids, marker_corners, marker_ids = \
                self.detector.detectBoard(gray)

            if corners is not None and ids is not None and len(ids) >= 8:
                all_corners.append(corners)
                all_ids.append(ids)
                captured_frames += 1
                self.get_logger().info(
                    f"Captured frame {captured_frames}/{self.max_frames} "
                    f"({len(ids)} corners detected)")

                aruco.drawDetectedCornersCharuco(
                    frame, corners, ids, (0, 255, 0))
                aruco.drawDetectedMarkers(
                    frame, marker_corners, marker_ids)
                self._publish_annotated(frame)
            else:
                num_ids = len(ids) if ids is not None else 0
                self.get_logger().info(
                    f"Board not detected (corners: {num_ids}, "
                    f"need >= 8), skipping frame")
                self._publish_annotated(frame)

            # Rate limit
            elapsed = time.perf_counter() - frame_start
            sleep_time = (1.0 / self.frame_rate_hz) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._perform_calibration(
            all_corners, all_ids, image_size, camera_serial)

    def _publish_annotated(self, frame):
        """Publish an annotated frame as a compressed image."""
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(
                frame, dst_format='jpg')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().warning(
                f"Failed to publish annotated image: {e}")

    def _perform_calibration(self, all_corners, all_ids, image_size,
                             camera_serial):
        """Compute camera intrinsics from collected charuco frames."""
        self.get_logger().info(
            f"Performing calibration with {len(all_corners)} frames...")

        all_obj_points = []
        all_img_points = []

        for c, i in zip(all_corners, all_ids):
            objp, imgp = self.board.matchImagePoints(c, i)
            all_obj_points.append(objp)
            all_img_points.append(imgp)

        rms, mtx, dist, _, _ = cv2.calibrateCamera(
            objectPoints=all_obj_points,
            imagePoints=all_img_points,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )

        calib_results = {
            "matrix": mtx.tolist(),
            "disto": dist.tolist(),
            "rmse_reprojection_error": rms,
            "method": "charuco",
            "board_cols": self.board_cols,
            "board_rows": self.board_rows,
            "square_length": self.square_length,
            "marker_length": self.marker_length,
        }

        output_file = os.path.join(
            os.getcwd(), f'calibrationmatrix_{camera_serial}.json')
        with open(output_file, 'w') as f:
            json.dump(calib_results, f, indent=4)

        self.get_logger().info(
            f"Calibration saved to {output_file} (RMSE: {rms:.4f})")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CharucoCalibrationNode2()
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
