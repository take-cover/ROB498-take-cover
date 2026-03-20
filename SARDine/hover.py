import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.qos import qos_profile_system_default
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
from tf2_geometry_msgs import do_transform_pose

class HoverNode(Node):
    def __init__(self):
        super().__init__('hover_node')

        # Subscriber for the Tag's position relative to the camera 
        self.aruco_sub = self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.aruco_pose_callback,
            qos_profile_system_default
        )

        # Subscriber for the Drone's Pose captured at the moment of detection 
        self.drone_detection_sub = self.create_subscription(
            PoseStamped,
            '/aruco/drone_pose_at_detection',
            self.calculate_global_tag_callback,
            qos_profile_system_default
        )

        self.init_vicon_pose_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.initial_vicon_pose_callback,
            qos_profile_system_default
        )

        self.initial_vicon_pose = None
        self.latest_tag_camera_pos = None
        
        # Publisher for the final calculated world coordinate of the tag
        self.world_tag_pub = self.create_publisher(
            PoseStamped, 
            '/aruco/world_pose', 
            qos_profile_system_default
        )

    def aruco_pose_callback(self, msg):
        """Stores the relative camera vector: P_tag_camera"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose

        self.latest_tag_camera_pos = current_pose
        self.get_logger().info(f"Received ArUco pose: x={self.latest_tag_camera_pos.pose.position.x}, y={self.latest_tag_camera_pos.pose.position.y}, z={self.latest_tag_camera_pos.pose.position.z}")
    
    def initial_vicon_pose_callback(self, msg):
        """Update initial pose from Vicon"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose

        # Update initial vicon pose
        if self.initial_vicon_pose is None:
            self.initial_vicon_pose = current_pose
            self.get_logger().info(f"Set initial Vicon pose: x={self.initial_vicon_pose.pose.position.x}, y={self.initial_vicon_pose.pose.position.y}, z={self.initial_vicon_pose.pose.position.z}")

    def calculate_global_tag_callback(self, drone_detection_pose_msg):
        """
        Triggers when the 'snapshot' drone pose is received.
        Performs the 3D transformation to the Vicon frame.
        """
        if self.latest_tag_camera_pos is None:
            self.get_logger().warn("Drone pose received, but no tag detection available!")
            return

        # Drone orientation in Vicon frame at the moment of detection
        q_drone = [
            drone_detection_pose_msg.pose.orientation.x,
            drone_detection_pose_msg.pose.orientation.y,
            drone_detection_pose_msg.pose.orientation.z,
            drone_detection_pose_msg.pose.orientation.w
        ]

        q_180x = (1, 0, 0, 0)  # quaternion for 180 degree rotation about X-axis

        # Determine the combined rotation
        q_combined = quaternion_multiply(q_drone, q_180x)

        # Change position vector into a pure quaternion (x, y, z, 0) for translation computation 
        latest_pose_array = np.array([
            self.latest_tag_camera_pos.pose.position.x,
            self.latest_tag_camera_pos.pose.position.y,
            self.latest_tag_camera_pos.pose.position.z
        ])
        tag_pos_cam = latest_pose_array + [0.0]

        # Get inverse quaternion for inverse rotation
        q_conjugate = quaternion_inverse(q_combined)

        # Get tag position in world frame 
        tag_ori_world = quaternion_multiply(q_combined, tag_pos_cam)
        tag_ori_world = quaternion_multiply(tag_ori_world, q_conjugate)

        # Extract (x, y, z) for translation computation from current drone location
        tag_pos_cam = tag_ori_world[:3]

        p_drone = [
            drone_detection_pose_msg.pose.position.x,
            drone_detection_pose_msg.pose.position.y,
            drone_detection_pose_msg.pose.position.z
        ]
        # Final tag position in world frame
        tag_pos_world = np.array(p_drone) + np.array(tag_pos_cam)

        # Publish the result 
        initial_pose_vector = np.array([
            self.initial_vicon_pose.pose.orientation.x,
            self.initial_vicon_pose.pose.orientation.y,
            self.initial_vicon_pose.pose.orientation.z,
            self.initial_vicon_pose.pose.orientation.w
        ])
        world_msg = PoseStamped()
        world_msg.header.stamp = self.get_clock().now().to_msg()
        world_msg.header.frame_id = "map" 
        world_msg.pose.position.x = tag_pos_world[0]
        world_msg.pose.position.y = tag_pos_world[1]
        world_msg.pose.position.z = tag_pos_world[2]
        world_msg.pose.orientation.x = initial_pose_vector[0]
        world_msg.pose.orientation.y = initial_pose_vector[1]
        world_msg.pose.orientation.z = initial_pose_vector[2]
        world_msg.pose.orientation.w = initial_pose_vector[3]
        self.world_tag_pub.publish(world_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()