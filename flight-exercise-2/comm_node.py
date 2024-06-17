import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool

G_HEIGHT = 1.5

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1')

        # publishers
        self.ego_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.create_timer(5, self.publish_position) # publish vision pose at 20Hz

        # subscribers
        self.vicon_sub = self.create_subscription( # not using if no vicon...
            PoseStamped, 
            '/vicon/ROB498_Drone/ROB498_Drone', 
            self.vicon_callback,
            10
        )
        self.realsense_sub = self.create_subscription(
            Odometry,
            "/camera/pose/sample",
            self.realsense_callback,
            qos_profile_system_default
        )


        # services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)

        # pose
        self.initial_pose = None
        self.latest_pose = None

        # state
        self.state = State()
        # mavros clients
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")

        self.get_logger().info("CommNode initiailized, has not received intial pose yet.")

    # ---------------------------- commands -------------------------------------------------
    def cmd_arm_drone(self, arm_status):
        if self.arming_client.service_is_ready():
            print("arming drone")
            req = CommandBool.Request()
            req.value = arm_status
            self.arming_client.call_async(req)
        else:
            print("client service not ready")


    def callback_launch(self, request, response):
        """Handle LAUNCH command: take off to desired height above initial pose"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response
        
        # target_z = self.initial_pose.position.z + G_HEIGHT

        # self.get_logger().info(f"Launch Requested. Target altitude: {G_HEIGHT}m")

        # arm drone if not armed yet
        if not self.state.armed:
            print("calling cmd_arm_drone")
            self.cmd_arm_drone(True)

        response.success = True
        response.message = "Drone taking off."
        return response

    def callback_test(self, request, response):
        """Handle TEST command: just wait until TA done collecting data..."""
        self.get_logger().info("Test Requested. Starting test sequence.")
        response.success = True
        response.message = "Test has started. Recording data."
        return response
    
    def callback_land(self, request, response):
        """Handle LAND command: descend back to intial altitude"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response

        self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z-0.05)
        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        response.success = True
        response.message = "Drone is landing."
        return response

    def callback_abort(self, request, response):
        """Handle ABORT command"""
        self.get_logger().warn("ABORT Requested! Cutting all thrust.")
        if self.initial_pose is None:
            response.success = False
            response.message = "Initial pose not received yet."
            return response

        self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        response.success = True
        '''
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "map"
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.linear.y = 0.0
        stop_msg.twist.linear.z = 0.0
        stop_msg.twist.angular.x = 0.0
        stop_msg.twist.angular.y = 0.0
        stop_msg.twist.angular.z = 0.0

        self.shutdown_publisher.publish(stop_msg)

        response.success = True
        '''
        response.message = "Emergency shutdown command sent. Drone should land immediately."
        return response

    def realsense_callback(self, msg):
        """Update pose from RealSense"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose

        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info("Realsense: set initial pose")
        self.latest_pose = current_pose

    def vicon_callback(self, msg):
        """Store initial pose"""
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
            self.get_logger().info(f"Initial pose recorded and set: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}, z={self.initial_pose.position.z}")

    def publish_position(self):
        """Publishes a new desired position."""
        if self.latest_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            # pos_msg = PoseStamped()
            pos_msg = self.latest_pose # PoseStamped object
            pos_msg.header.stamp = self.get_clock().now().to_msg()
            pos_msg.header.frame_id = "map"
            self.ego_pos_pub.publish(pos_msg)
            self.get_logger().info(f"Published self pose")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()