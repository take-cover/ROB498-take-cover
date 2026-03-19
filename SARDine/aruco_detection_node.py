import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import qos_profile_system_default


# Camera intrinsics for IMX219 at 1280x720 - replace with your calibrated values
CAMERA_MATRIX = np.array([
    [820.0,   0.0, 640.0],
    [  0.0, 820.0, 360.0],
    [  0.0,   0.0,   1.0]
])
DIST_COEFFS = np.zeros((4, 1))  # replace with calibrated values
MARKER_SIZE = 0.1  # [m] physical size of your printed marker

DEBUG_ON = True


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_system_default
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/marker_pose',
            qos_profile_system_default
        )
        if DEBUG_ON:
            self.debug_pub = self.create_publisher(
                Image,
                '/aruco/debug_image',
                qos_profile_system_default
            )

        self.obj_points = np.array([
            [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
            [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
            [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
            [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
        ], dtype=np.float32)
        
        self.get_logger().info("ArUco detection node initialized.")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        print(corners, ids)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                _, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corners[i][0], CAMERA_MATRIX, DIST_COEFFS
                )
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera"
                pose_msg.pose.position.x = float(tvec[0][0])
                pose_msg.pose.position.y = float(tvec[1][0])
                pose_msg.pose.position.z = float(tvec[2][0])
                # Default quaternion (no rotation)
                pose_msg.pose.orientation.w = 1.0 
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                self.pose_pub.publish(pose_msg)

                # self.get_logger().info(
                #     f"Marker {marker_id}: x={tvec[0][0][0]:.2f}, "
                #     f"y={tvec[0][0][1]:.2f}, z={tvec[0][0][2]:.2f}"
                # )

                if DEBUG_ON:
                    # Draw on debug image
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 0.05)
                    cv2.putText(frame, f"ID: {marker_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"x: {tvec[0][0]:.3f}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"y: {tvec[1][0]:.3f}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"z: {tvec[2][0]:.3f}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    

        # Publish debug image to be viewable in rqt
        if DEBUG_ON:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()