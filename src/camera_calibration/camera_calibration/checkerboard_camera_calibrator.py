import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os


class CheckerboardCalibrationNode(Node):
    def __init__(self):
        super().__init__(
            'checkerboard_calibrator_node',
            allow_undeclared_parameters=True,
        )

        self.declare_parameter('board_cols', 9)  # interior corners
        self.declare_parameter('board_rows', 6)  # interior corners
        self.declare_parameter('square_length', 0.0254)  # in meters
        self.declare_parameter('subscriber_topic', '/camera/image_raw')
        self.declare_parameter('publisher_topic', '/calibration/image_annotated')
        self.declare_parameter('camera_serial', 'N/A')
        self.declare_parameter('max_frames', 30)

        self.board_cols = self.get_parameter('board_cols').value
        self.board_rows = self.get_parameter('board_rows').value
        self.square_length = self.get_parameter('square_length').value
        subscriber_topic = self.get_parameter('subscriber_topic').value
        publisher_topic = self.get_parameter('publisher_topic').value

        self.bridge = CvBridge()

        # Prepare object points
        self.objp = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        self.objp *= self.square_length

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(nanoseconds=50_000_000),  # 50 ms; optional
        )

        self.subscriber = self.create_subscription(
            Image,
            subscriber_topic,
            self.image_callback,
            qos)

        self.publisher = self.create_publisher(Image, publisher_topic, 10)

        self.obj_points = []
        self.img_points = []
        self.captured_frames = 0
        self.max_frames = self.get_parameter('max_frames').value

        self.calibration_done = False
        self.num_consecutive_frames_detected = 0
        self.calibrate_every_n_frames = 10

    def image_callback(self, msg):
        if self.calibration_done:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(
            gray,
            (self.board_cols, self.board_rows),
            cv2.CALIB_CB_FAST_CHECK
        )

        if ret:  # If chessboard corners are found
            self.num_consecutive_frames_detected += 1
            cv2.drawChessboardCorners(img, (self.board_cols, self.board_rows), corners, ret)

        if self.num_consecutive_frames_detected >=  self.calibrate_every_n_frames:
            self.get_logger().info("Chessboard detected successfully.")
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw the corners
            cv2.drawChessboardCorners(img, (self.board_cols, self.board_rows), corners2, ret)

            self.obj_points.append(self.objp)
            self.img_points.append(corners2)
            self.captured_frames += 1
            self.get_logger().info(f"Captured frame {self.captured_frames}/{self.max_frames}")
            self.num_consecutive_frames_detected = 0

        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(annotated_image_msg)

        if self.captured_frames >= self.max_frames:
            self.perform_calibration(gray.shape[::-1])
            self.calibration_done = True

    def perform_calibration(self, image_size):
        self.get_logger().info("Performing calibration...")

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            image_size,
            None,
            None
        )

        # Calculate reprojection error
        mean_error = 0
        for i in range(len(self.obj_points)):
            imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        total_error = mean_error / len(self.obj_points)

        calib_results = {
            "matrix": mtx.tolist(),
            "disto": dist.tolist(),
            "rmse_reprojection_error": float(total_error),
            "method": "checkerboard",
            "board_cols": self.board_cols,
            "board_rows": self.board_rows,
            "square_length": self.square_length,
        }

        camera_serial = self.get_parameter('camera_serial').value

        output_file = os.path.join(os.getcwd(), f'calibrationmatrix_{camera_serial}.json')
        with open(output_file, 'w') as f:
            json.dump(calib_results, f, indent=4)

        self.get_logger().info(
            f"Calibration results saved to {output_file}. Press ctrl-c to exit.")


def main(args=None):
    rclpy.init(args=args)
    calibration_node = CheckerboardCalibrationNode()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
