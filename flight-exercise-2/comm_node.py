import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


HOVER_Z = 0.5 # [m]
FREQ_30_HZ = 1/30 # [1/Hz]
FREQ_0_5_HZ = 2 # [1/Hz]
FREQ_10_HZ = 1/10 # [1/Hz]
LANDING_TOL = 0.1 # [m]
LANDED_TOL = 0.005 # [m]
LOG_LATEST_POSE = True
VICON_DRONE_GND_Z = 0.18 # [m] UPDATE WITH REAL VICON MEASUREMENT

LOG_LATEST_POSE = True
LOG_WAYPOINT = False


class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1')
        # Poses
        self.initial_pose = None # Startup pose (first power on)
        self.latest_pose = None

        self.waypoint_pose = PoseStamped() # Pose to hold during test
        self.test_state = 0 # 1 = hold position during test
        
        self.use_vicon = False
        self.state = State()

        # Set up publishers
        self.ego_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_position) # publish vision pose at 30Hz
        self.create_timer(FREQ_0_5_HZ, self.print_position)

        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_waypoint) # publish waypoint at 30Hz
        self.create_timer(FREQ_0_5_HZ, self.print_waypoint)
        
        # Set up subscribers
        if self.use_vicon:
            self.vicon_sub = self.create_subscription(
                PoseStamped, 
                '/vicon/ROB498_Drone/ROB498_Drone', 
                self.vicon_callback,
                10
            )
        else:
            self.realsense_sub = self.create_subscription(
                Odometry,
                "/camera/pose/sample",
                self.realsense_callback,
                qos_profile_system_default
            )

        # Set up mavros clients
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # Create services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)

        self.get_logger().info("CommNode initiailized; initial pose not yet received.")


    ############################################################################
    # Drone commands
    ############################################################################
    def arm_drone(self, arm_status):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = arm_status
            self.arming_client.call_async(req)
            print("Drone armed")
        else:
            print("Arming client not ready")


    def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)
            print(f"Set mode to {mode}")
        else:
            print("Set mode client not ready")


    ############################################################################
    # Service callbacks
    ############################################################################
    def callback_launch(self, request, response):
        """Handle LAUNCH command: take off to desired height above initial pose"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response
        
        # Optional manual takeoff
        # self.set_mode("ALTCTL")
        # self.get_logger().info("Initiate manual takeoff.")
       
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

        # arm drone if not armed yet
        if not self.state.armed:
            print("Arming drone...")
            self.arm_drone(True)
            
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = self.initial_pose.pose.position.x
        target_pose.pose.position.y = self.initial_pose.pose.position.y
        target_pose.pose.position.z = self.initial_pose.pose.position.z + HOVER_Z - VICON_DRONE_GND_Z # set target to hover at desired height above initial position

        target_pose.pose.orientation.x = self.initial_pose.pose.orientation.x
        target_pose.pose.orientation.y = self.initial_pose.pose.orientation.y
        target_pose.pose.orientation.z = self.initial_pose.pose.orientation.z
        target_pose.pose.orientation.w = self.initial_pose.pose.orientation.w       

        self.get_logger().info(f"Launch Requested. Target altitude: {target_pose.pose.position.z}m")
        self.waypoint_pose = target_pose

        response.success = True
        response.message = "Drone taking off."

        return response


    def callback_test(self, request, response):
        """Handle TEST command: Set waypoint & wait until TA done collecting data..."""
        self.get_logger().info("Test Requested. Starting test sequence.")

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            
        if self.test_state == 0:
            self.waypoint_pose = self.latest_pose
            self.test_state = 1 # set to fixed position (position at the start of test call)

        response.success = True
        response.message = "Test has started. Recording data."
        return response
    

    def callback_land(self, request, response):
        """Handle LAND command: descend back to initial altitude"""        
        self.set_mode("AUTO.LAND")

        response.success = True
        response.message = "Drone is landing."
        return response


    def callback_abort(self, request, response):
        """
        Handle ABORT command: descend back to intiial altitude
        CURRENTLY: hands over control to manual mode
        """
        self.callback_land(request, response)

        response.success = True
        response.message = "Drone aborted."
        return response


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
        
        self.latest_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(f"Vicon - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")


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


    def publish_waypoint(self):
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint_pose)
        if LOG_WAYPOINT:
            self.get_logger().info(f"Published waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")


    ############################################################################
    # Print Functions
    ############################################################################
    def print_position(self):
        self.get_logger().info(f"Realsense: latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")


    def print_waypoint(self):
        self.get_logger().info(f"Waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")
        

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
