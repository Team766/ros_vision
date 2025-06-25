import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import json
import os

class CameraCalibrationNode(Node):
    def __init__(self):

        super().__init__(
            'camera_calibrator_node',
            allow_undeclared_parameters=True,
        )

        self.declare_parameter('board_cols', 11)
        self.declare_parameter('board_rows', 8)
        self.declare_parameter('square_length', 0.015)
        self.declare_parameter('marker_length', 0.011)
        self.declare_parameter('subscriber_topic', '/camera/image_raw')
        self.declare_parameter('publisher_topic', '/calibration/image_annotated')
        self.declare_parameter('camera_serial', 'N/A')
        self.declare_parameter('max_frames', 30)

        self.board_cols = self.get_parameter('board_cols').value
        self.board_rows = self.get_parameter('board_rows').value
        square_length = self.get_parameter('square_length').value
        marker_length = self.get_parameter('marker_length').value
        subscriber_topic = self.get_parameter('subscriber_topic').value
        publisher_topic = self.get_parameter('publisher_topic').value

        self.bridge = CvBridge()

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.board = aruco.CharucoBoard(
            (self.board_cols, self.board_rows),
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=self.dictionary
        )
        self.detector = aruco.CharucoDetector(self.board)

        self.subscriber = self.create_subscription(
            Image,
            subscriber_topic,
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Image, publisher_topic, 10)

        self.all_corners = []
        self.all_ids = []
        self.captured_frames = 0
        self.max_frames = self.get_parameter('max_frames').value

        self.calibration_done = False

    def image_callback(self, msg):
        if self.calibration_done:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        corners, ids, marker_corners, marker_ids = self.detector.detectBoard(img)

        if corners is not None and ids is not None and len(ids) >= 16:
            aruco.drawDetectedCornersCharuco(img, corners, ids, (0, 255, 0))
            aruco.drawDetectedMarkers(img, marker_corners, marker_ids)

            self.all_corners.append(corners)
            self.all_ids.append(ids)
            self.captured_frames += 1
            self.get_logger().info(f"Captured frame {self.captured_frames}/{self.max_frames}")

        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(annotated_image_msg)

        if self.captured_frames >= self.max_frames:
            self.perform_calibration(img.shape[:2][::-1])
            self.calibration_done = True

    def perform_calibration(self, image_size):
        self.get_logger().info("Performing calibration...")
        all_obj_points = []
        all_img_points = []

        for c, i in zip(self.all_corners, self.all_ids):
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
            "distortion": dist.tolist(),
            "rmse_reprojection_error": rms
        }

        camera_serial = self.get_parameter('camera_serial').value

        output_file = os.path.join(os.getcwd(), f'calibrationmatrix_{camera_serial}.json')
        with open(output_file, 'w') as f:
            json.dump(calib_results, f, indent=4)

        self.get_logger().info(f"Calibration results saved to {output_file}.  Press ctrl-c to exit.")

def main(args=None):
    rclpy.init(args=args)
    calibration_node = CameraCalibrationNode()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
