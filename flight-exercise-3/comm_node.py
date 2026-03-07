import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PoseArray
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
VICON_DRONE_GND_Z = 0.18 # [m] UPDATE WITH REAL VICON MEASUREMENT

LOG_LATEST_POSE = True
LOG_WAYPOINT = False

WAYPOINTS = None
WAYPOINTS_RECEIVED = False

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1')
        # Poses
        self.initial_pose = None # Startup pose (first power on)
        self.latest_pose = None

        self.waypoint_pose = PoseStamped() # Pose to hold during test
        
        self.use_vicon = False
        self.state = State()

        # Set up publishers
        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_waypoint) # publish waypoint at 30Hz
        self.create_timer(FREQ_0_5_HZ, self.print_waypoint)
        
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
        self.sub_waypoints = self.create_subscription(
            PoseArray, 
            'rob498_drone_1/comm/waypoints', 
            self.callback_waypoints, 
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
            self.get_logger().info("Drone armed")
        else:
            self.get_logger().info(f"arming_client not ready; cannot arm drone")


    def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)
            self.get_logger().info(f"Set mode to {mode}")
        else:
            self.get_logger().info(f"set_mode_client not ready; cannot change mode to {mode}")


    ############################################################################
    # Service callbacks
    ############################################################################
    def callback_launch(self, request, response):
        """Handle LAUNCH command: take off to desired height above initial pose"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response
       
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

        if not self.state.armed:
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
        if not WAYPOINTS_RECEIVED:
            response.success = False
            response.message = "Waypoints not received."
            return response

        self.get_logger().info("Test Requested. Starting test sequence.")

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            
        self.waypoint_pose = self.latest_pose

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
        

    ############################################################################
    # Waypoints callback (TODO)
    ############################################################################
    def callback_waypoints(self, msg):
        global WAYPOINTS_RECEIVED, WAYPOINTS
        if WAYPOINTS_RECEIVED:
            return
        print('Waypoints Received')
        WAYPOINTS_RECEIVED = True
        WAYPOINTS = np.empty((0,3))
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            WAYPOINTS = np.vstack((WAYPOINTS, pos))


    ############################################################################
    # Publisher functions
    ############################################################################
    def publish_waypoint(self):
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint_pose)
        if LOG_WAYPOINT:
            self.get_logger().info(f"Published waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")


    ############################################################################
    # Print Functions
    ############################################################################
    def print_waypoint(self):
        self.get_logger().info(f"Waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")
        

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
