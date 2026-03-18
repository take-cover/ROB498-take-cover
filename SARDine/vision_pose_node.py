import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix

from tf2_geometry_msgs import do_transform_pose


FREQ_30_HZ = 1/30 # [1/Hz]
FREQ_0_5_HZ = 2 # [1/Hz]

USE_VICON = False

LOG_LATEST_POSE = True
    

class MavrosVisionPoseNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1_vision_pose_node')       
        # Init pose variables
        self.initial_cam_pose = None # Startup pose (in camera frame)
        self.initial_vicon_pose = None # Startup pose (in Vicon frame)
        
        self.latest_pose = None # Always in Vicon frame
        
        self.use_vicon = USE_VICON
        
        # Init camera to Vicon Transform
        self.cam_to_vicon_tf = None
        
        # Set up publishers
        self.init_cam_pose_pub = self.create_publisher(
            PoseStamped,
            "/team1_fe3/vision_pose/initial_cam_pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_initial_cam_pose)
        
        self.init_vicon_pose_pub = self.create_publisher(
            PoseStamped,
            "/team1_fe3/vision_pose/initial_vicon_pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_initial_vicon_pose)
        
        self.ego_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_pose)
        # self.create_timer(FREQ_0_5_HZ, self.print_pose)

        self.cam_to_vicon_tf_pub = self.create_publisher(
            TransformStamped,
            "/team1_fe3/cam_to_vicon_tf",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_cam_to_vicon_tf)
        
        # Set up subscribers
        self.init_vicon_pose_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.initial_vicon_pose_callback,
            qos_profile_system_default
        )
 
        if self.use_vicon:
            self.vicon_sub = self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone',
                self.vicon_callback,
                qos_profile_system_default
            )
        else:
            self.realsense_sub = self.create_subscription(
                Odometry,
                "/camera/pose/sample",
                self.realsense_callback,
                qos_profile_system_default
            )
            
        if not self.use_vicon:
            # Calculate camera to Vicon frame transform using initial poses
            self.cam_to_vicon_tf_timer = self.create_timer(FREQ_0_5_HZ, self.calculate_cam_to_vicon_tf)
            
        self.get_logger().info("MavrosVisionPoseNode initialized; initial pose not yet received.")

    ############################################################################
    # External pose update callbacks (camera & Vicon)
    ############################################################################
    def realsense_callback(self, msg):
        """Update pose from RealSense"""    
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "camera"
        current_pose.pose = msg.pose.pose
        
        # Rotate pose from camera by 180 degrees in yaw (flip x and y axes)
        current_pose.pose.position.x *= -1
        current_pose.pose.position.y *= -1

        # Update pose(s)
        if self.initial_cam_pose is None:
            self.initial_cam_pose = current_pose
            self.get_logger().info(
                f"Realsense - Set initial camera pose: x={self.initial_cam_pose.pose.position.x}, y={self.initial_cam_pose.pose.position.y}, z={self.initial_cam_pose.pose.position.z}"
            )
            
        if self.cam_to_vicon_tf is not None:
            current_pose = do_transform_pose(current_pose, self.cam_to_vicon_tf)
            self.latest_pose = current_pose
        
        if LOG_LATEST_POSE and self.latest_pose is not None:
            self.get_logger().info(
                f"Latest pose (source = Realsense): x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}"
            )
    
    def vicon_callback(self, msg):
        """Update pose from Vicon"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose
        
        # Update pose; let initial Vicon pose be set in initial_vicon_pose_callback
        if self.initial_vicon_pose is None:
            return
        
        self.latest_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(
                f"Latest pose (source = Vicon): x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}"
            )

    def calculate_cam_to_vicon_tf(self):
        """Compute transform T_vicon_camera from initial poses."""
        if self.initial_cam_pose is None or self.initial_vicon_pose is None or \
            self.cam_to_vicon_tf is not None:
            return None

        # Get initial poses for camera and vicon
        q_v = self.initial_vicon_pose.pose.orientation
        p_v = self.initial_vicon_pose.pose.position
        
        q_c = self.initial_cam_pose.pose.orientation
        p_c = self.initial_cam_pose.pose.position

        # Compute transform from camera to Vicon
        # Rotation
        q_v_tuple = (q_v.x, q_v.y, q_v.z, q_v.w)
        q_c_tuple = (q_c.x, q_c.y, q_c.z, q_c.w)
        q_c_inv = quaternion_inverse(q_c_tuple)
        q_vc = quaternion_multiply(q_v_tuple, q_c_inv)
        # Translation
        t_c = np.array([p_c.x, p_c.y, p_c.z])
        t_v = np.array([p_v.x, p_v.y, p_v.z])
        R_vc = quaternion_matrix(q_vc)[:3, :3]
        t_vc = t_v - R_vc @ t_c

        # Create TransformStamped message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "vicon"
        tf_msg.child_frame_id = "camera"
        tf_msg.transform.translation.x = float(t_vc[0])
        tf_msg.transform.translation.y = float(t_vc[1])
        tf_msg.transform.translation.z = float(t_vc[2])
        tf_msg.transform.rotation.x = float(q_vc[0])
        tf_msg.transform.rotation.y = float(q_vc[1])
        tf_msg.transform.rotation.z = float(q_vc[2])
        tf_msg.transform.rotation.w = float(q_vc[3])

        self.cam_to_vicon_tf = tf_msg
        self.cam_to_vicon_tf_timer.cancel() # Only need to calculate once
        self.get_logger().info(
            f"Calculated camera to Vicon transform: translation=({tf_msg.transform.translation.x}, {tf_msg.transform.translation.y}, {tf_msg.transform.translation.z}), "
            f"rotation=({tf_msg.transform.rotation.x}, {tf_msg.transform.rotation.y}, {tf_msg.transform.rotation.z}, {tf_msg.transform.rotation.w})"
        )
    
    
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

    ############################################################################
    # Publisher functions
    ############################################################################
    def publish_pose(self):
        """Publishes a new desired position."""
        if self.latest_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_pub.publish(self.latest_pose)


    def publish_initial_cam_pose(self):
        if self.initial_cam_pose is None:
            self.get_logger().info("Initial pose not registered, nothing to publish")
        else:
            self.initial_cam_pose.header.stamp = self.get_clock().now().to_msg()
            self.init_cam_pose_pub.publish(self.initial_cam_pose)


    def publish_initial_vicon_pose(self):
        if self.initial_vicon_pose is None:
            self.get_logger().info("Initial Vicon pose not registered, nothing to publish")
        else:
            self.initial_vicon_pose.header.stamp = self.get_clock().now().to_msg()
            self.init_vicon_pose_pub.publish(self.initial_vicon_pose)


    def publish_cam_to_vicon_tf(self):
        if self.cam_to_vicon_tf is None:
            self.get_logger().info("Camera to Vicon transform not yet calculated, nothing to publish")
        else:
            self.cam_to_vicon_tf.header.stamp = self.get_clock().now().to_msg()
            self.cam_to_vicon_tf_pub.publish(self.cam_to_vicon_tf)

    ############################################################################
    # Print Functions
    ############################################################################
    def print_pose(self):
        if self.latest_pose is not None:
            self.get_logger().info(
                f"Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MavrosVisionPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
