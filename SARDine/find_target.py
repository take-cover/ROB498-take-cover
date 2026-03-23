import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import qos_profile_system_default
from scipy.spatial.transform import Rotation as R

# Transform from RGB camera to the Vicon marker frame:
# 180 deg rotation about X (wrt RGB camera frame)
# Translations in x, y, z (wrt Vicon marker frame)
t_vec = np.array([-0.1, 0.03, -0.05])  # [m] replace with measured translation
RGB_TO_MARKER_TRANSFORM = np.array([
    [1, 0, 0,   t_vec[0]],
    [0, -1, 0,  t_vec[1]],
    [0, 0, -1,  t_vec[2]],
    [0, 0, 0,   1]
])
TIMER_10_HZ = 1/30 # [1/Hz]
VALID_POSE_TOL_S = 0.2 # [s]
DEBUG_ON = True

class TrackandHoverNode(Node):
    def __init__(self):
        super().__init__('track_and_hover')
        self.latest_aruco_pose = None
        self.latest_drone_pose = None

        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose',
            self.aruco_pose_callback,
            qos_profile_system_default
        )
        
        self.drone_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.drone_pose_callback,
            qos_profile_system_default
        )
        
        self.create_timer(TIMER_10_HZ, self.track_aruco_callback)


    def aruco_pose_callback(self, msg):
        latest_pose = PoseStamped()
        latest_pose.header.stamp = self.get_clock().now().to_msg()
        latest_pose.header.frame_id = "rgb_camera"
        latest_pose.pose = msg.pose
        self.latest_aruco_pose = latest_pose


    def drone_pose_callback(self, msg):
        latest_pose = PoseStamped()
        latest_pose.header.stamp = self.get_clock().now().to_msg()
        latest_pose.header.frame_id = "drone_vicon_marker"
        latest_pose.pose = msg.pose
        self.latest_drone_pose = latest_pose
        

    def track_aruco_callback(self):
        if self.latest_aruco_pose is None or self.latest_drone_pose is None or \
        (self.get_clock().now() - self.latest_aruco_pose.header.stamp).nanoseconds * 1e-9 > VALID_POSE_TOL_S or \
        (self.get_clock().now() - self.latest_drone_pose.header.stamp).nanoseconds * 1e-9 > VALID_POSE_TOL_S:
            return
        r_pg_g = np.array([
            [self.latest_aruco_pose.pose.position.x],
            [self.latest_aruco_pose.pose.position.y],
            [self.latest_aruco_pose.pose.position.z],
            [1]
        ])
        r_pm_m = RGB_TO_MARKER_TRANSFORM @ r_pg_g
        
        r_mv_v = np.array([
            [self.latest_drone_pose.pose.position.x],
            [self.latest_drone_pose.pose.position.y],
            [self.latest_drone_pose.pose.position.z],
            [1]
        ])
        q_vm = [
            self.latest_drone_pose.pose.orientation.x,
            self.latest_drone_pose.pose.orientation.y,
            self.latest_drone_pose.pose.orientation.z,
            self.latest_drone_pose.pose.orientation.w
        ]
        R_vm = R.from_quat(q_vm).as_matrix()
        T_vm = np.eye(4)
        T_vm[:3, :3] = R_vm
        T_vm[:3, 3] = r_mv_v[:3, 0]
        
        r_pm_v = T_vm @ r_pm_m
        r_pv_v = r_mv_v + r_pm_v
        
        
        if DEBUG_ON:
            self.get_logger().info(f"Relative position of ArUco marker w.r.t. drone: x={r_pm_m[0][0]:.2f} m, y={r_pm_m[1][0]:.2f} m, z={r_pm_m[2][0]:.2f} m")

    
def main(args=None):
    rclpy.init(args=args)
    node = TrackandHoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()