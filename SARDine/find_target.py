import rclpy, rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.qos import qos_profile_system_default
from scipy.spatial.transform import Rotation as R

# Transform from RGB camera to the Vicon marker frame:
# 180 deg rotation about X (wrt RGB camera frame)
# Translations in x, y, z (wrt Vicon marker frame)
r_cm_m = np.array([-0.16, 0.035, -0.11])  # [m] replace with measured translation
T_MC = np.array([
    [1, 0, 0,   r_cm_m[0]],
    [0, -1, 0,  r_cm_m[1]],
    [0, 0, -1,  r_cm_m[2]],
    [0, 0, 0,   1]
])
TIMER_10_HZ = 1/10 # [1/Hz]
VALID_POSE_TOL_S = 0.2 # [s]

USE_VICON_Z = True # whether to use Vicon z or measured Z from RGB camera for Aruco tag distance
ARUCO_TAG_Z_HEIGHT = 0.1 # [m] height of the ArUco tag above the ground, used if USE_VICON_Z is False

DEBUG_ON = True


class TargetFinderNode(Node):
    def __init__(self):
        super().__init__('target_finder')
        self.latest_aruco_pose = None
        self.latest_drone_pose = None

        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            'take_cover/aruco/marker_pose',
            self.aruco_pose_callback,
            qos_profile_system_default
        )
        
        self.drone_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.drone_pose_callback,
            qos_profile_system_default
        )
        
        self.create_timer(TIMER_10_HZ, self.find_target_callback)
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            'take_cover/target/position',
            qos_profile_system_default
        )


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
        

    def find_target_callback(self):
        '''
        Get the position of the Aruco marker in the inertial Vicon frame
        '''
        if self.latest_aruco_pose is None or self.latest_drone_pose is None or \
        (self.get_clock().now() - rclpy.time.Time.from_msg(self.latest_aruco_pose.header.stamp)).nanoseconds * 1e-9 > VALID_POSE_TOL_S or \
        (self.get_clock().now() - rclpy.time.Time.from_msg(self.latest_drone_pose.header.stamp)).nanoseconds * 1e-9 > VALID_POSE_TOL_S:
            return
        
        # T1: static from RGB camera frame to Vicon marker frame
        r_pc_c = np.array([
            [self.latest_aruco_pose.pose.position.x],
            [self.latest_aruco_pose.pose.position.y],
            [self.latest_aruco_pose.pose.position.z if not USE_VICON_Z else \
             self.latest_drone_pose.pose.position.z - r_cm_m[2]] - ARUCO_TAG_Z_HEIGHT, 
            [1]
        ])
        r_pm_m = T_MC @ r_pc_c
        
        # T2: dynamic from Vicon marker frame to global Vicon frame
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
        r_pv_v = T_vm @ r_pm_m
        
        # Publish target position in the global Vicon frame
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "map"
        target_msg.pose.position.x = float(r_pv_v[0][0])
        target_msg.pose.position.y = float(r_pv_v[1][0])
        target_msg.pose.position.z = float(r_pv_v[2][0])
        # Default quaternion (no rotation)
        target_msg.pose.orientation.x = 0.0
        target_msg.pose.orientation.y = 0.0
        target_msg.pose.orientation.z = 0.0
        target_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(target_msg)
        
        if DEBUG_ON:
            self.get_logger().info(f"Camera: x={r_pc_c[0][0]:.2f} m, y={r_pc_c[1][0]:.2f} m, z={r_pc_c[2][0]:.2f} m")
            self.get_logger().info(f"Drone : x={r_pm_m[0][0]:.2f} m, y={r_pm_m[1][0]:.2f} m, z={r_pm_m[2][0]:.2f} m")
            self.get_logger().info(f"Vicon : x={r_pv_v[0][0]:.2f} m, y={r_pv_v[1][0]:.2f} m, z={r_pv_v[2][0]:.2f} m")

    
def main(args=None):
    rclpy.init(args=args)
    node = TargetFinderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()