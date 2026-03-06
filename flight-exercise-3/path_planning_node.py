import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1_path_planning_node')
        # Poses
        self.initial_pose = None
        self.latest_pose = None

        # Set up subscribers
        self.ego_sub = self.create_subscription(
            PoseStamped,
            "/mavros/vision_pose/pose",
            self.mavros_vision_pose_callback,
            qos_profile_system_default
        )
        self.ego_init_sub = self.create_subscription(
            PoseStamped,
            "/team1_fe3/vision_pose/initial_pose",
            self.mavros_vision_initial_pose_callback,
            qos_profile_system_default
        )

    ############################################################################
    # Pose update callbacks (from MavrosVisionPoseNode)
    ############################################################################
    def mavros_vision_pose_callback(self, msg):
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose

        self.latest_pose = current_pose

    def mavros_vision_initial_pose_callback(self, msg):
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose

        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info(f"Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")
        
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
