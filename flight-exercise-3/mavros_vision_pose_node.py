import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default


FREQ_30_HZ = 1/30 # [1/Hz]
FREQ_0_5_HZ = 2 # [1/Hz]
FREQ_10_HZ = 1/10 # [1/Hz]

LOG_LATEST_POSE = False

class MavrosVisionPoseNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1_mavros_vision_pose_node')
        # Poses
        self.initial_pose = None # Startup pose (first power on)
        self.latest_pose = None

        self.vicon_initial_pose = None
        
        self.use_vicon = True

        # Set up publishers
        self.ego_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_position) # publish vision pose at 30Hz
        self.create_timer(FREQ_0_5_HZ, self.print_position)

        self.ego_init_pub = self.create_publisher(
            PoseStamped,
            "/team1_fe3/vision_pose/initial_pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_initial_position)

        self.vicon_ego_init_pub = self.create_publisher(
            PoseStamped,
            "team1_fe3/vision_pose/vicon_initial_pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_vicon_initial_position)
        
        # Set up subscribers
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
            self.vicon_initial_pose_sub = self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone', 
                self.vicon_initial_pose_callback,
                qos_profile_system_default
            )

        self.get_logger().info("MavrosVisionPoseNode initiailized; initial pose not yet received.")

    ############################################################################
    # External pose update callbacks (camera & Vicon)
    ############################################################################
    def realsense_callback(self, msg):
        """Update pose from RealSense"""    
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose
        
        # Rotate pose from camera by 180 degrees in yaw (flip x and y axes)
        current_pose.pose.position.x *= -1
        current_pose.pose.position.y *= -1

        # Update pose(s)
        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info(f"Realsense - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")
            
        self.latest_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(f"Realsense - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")
    
    def vicon_callback(self, msg):
        """Update pose from Vicon"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose
        
        # Update pose(s)
        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info(f"Vicon - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")
        
        if self.vicon_initial_pose is None:
            self.vicon_initial_pose = current_pose
            self.get_logger().info(f"Vicon Init Pose - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")

        self.latest_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(f"Vicon - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")

    def vicon_initial_pose_callback(self, msg):
        """Update Vicon initial pose"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose

        # Update initial vicon pose
        if self.vicon_initial_pose is None:
            self.vicon_initial_pose = current_pose
            self.get_logger().info(f"Vicon Init Pose - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")

    ############################################################################
    # Publisher functions
    ############################################################################
    def publish_position(self):
        """Publishes a new desired position."""
        if self.latest_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_pub.publish(self.latest_pose)

    def publish_initial_position(self):
        if self.initial_pose is None:
            self.get_logger().info("Initial pose not registered, nothing to publish")
        else:
            self.initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_init_pub.publish(self.initial_pose)

    def publish_vicon_initial_position(self):
        if self.vicon_initial_pose is None:
            self.get_logger().info("Initial Vicon pose not registered, nothing to publish")
        else:
            self.vicon_initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.vicon_ego_init_pub.publish(self.vicon_initial_pose)

    ############################################################################
    # Print Functions
    ############################################################################
    def print_position(self):
        self.get_logger().info(f"Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = MavrosVisionPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
