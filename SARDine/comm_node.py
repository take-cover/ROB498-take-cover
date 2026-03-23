import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PoseArray

from rclpy.qos import qos_profile_system_default

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import FSM
import utils

HOVER_Z = 0.5 # [m]
TIMER_30_HZ = 1/30 # [1/Hz]
TIMER_0_5_HZ = 2 # [1/Hz]
TIMER_10_HZ = 1/10 # [1/Hz]

LOG_LATEST_POSE = True
LOG_SETPOINT = False

SEARCH_WAYPOINTS = np.array([
    [1.0, 0.0, -1.0, 0.0],
    [0.0, 1.0, 0.0, -1.0],
    [HOVER_Z, HOVER_Z, HOVER_Z, HOVER_Z]    
])

WAYPOINT_REACH_TOL = 0.1  # [m]
WAYPOINT_HOLD_TIME = 0.5   # [s]


class CommNode(Node):
    """
    Handle main drone FSM logic and flying.
    Note that all positions referenced by CommNode are in the global Vicon frame.
    """
    def __init__(self):
        super().__init__('rob498_drone_1')
        # Init pose variables
        self.initial_pose = None # Startup pose in Vicon frame
        self.latest_pose = None # Always in Vicon frame
        
        self.setpoint_pose = PoseStamped() # Current setpoint
        self.state = State() # mavros related
        
        # Set up waypoint FSM
        self.create_timer(TIMER_10_HZ, self.run_waypoint_fsm)
        self.fsm_active = False
        self.fsm_waypoint_index = 0
        self.fsm_hold_start_time = None

        # Master-FSM
        self.master_fsm = FSM.State()
        self.master_fsm = FSM.State.IDLE
        FSM.service_call_launch_done(False)
        FSM.service_call_test_done(False)
        self.create_timer(TIMER_30_HZ, self.evaluate_state)

        # Set up publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            qos_profile_system_default
        )
        self.create_timer(TIMER_30_HZ, self.publish_setpoint) # publish waypoint at 30Hz
        self.create_timer(TIMER_0_5_HZ, self.print_setpoint)
        
        # Set up subscribers
        self.ego_sub = self.create_subscription(
            PoseStamped,
            "/mavros/vision_pose/pose",
            self.mavros_vision_pose_callback,
            qos_profile_system_default
        )
        self.marker_world_pose_sub = self.create_subscription(
            PoseStamped,
            "/team1_fe3/marker_world_pose",
            self.marker_world_pose_callback,
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

    def master_fsm_transition(self, event):
        new_state = FSM.transition(self.master_fsm, event)
        if new_state is None:
            self.get_logger().info(f"ERROR: Master FSM: Transition does not exist from state {self.master_fsm} with event {event}")
            new_state = self.master_fsm
        else:
            self.get_logger().info(f"Master FSM: Transitioned from state {self.master_fsm} to new state {new_state} via event {event}")
        self.master_fsm = new_state

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
        if not FSM.state_equal(self.master_fsm, FSM.State.IDLE):
            response.success = False
            response.message = f"Cannot launch from state {self.master_fsm}"
            return response

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
        target_pose.pose.position.z = HOVER_Z

        target_pose.pose.orientation.x = self.initial_pose.pose.orientation.x
        target_pose.pose.orientation.y = self.initial_pose.pose.orientation.y
        target_pose.pose.orientation.z = self.initial_pose.pose.orientation.z
        target_pose.pose.orientation.w = self.initial_pose.pose.orientation.w       

        self.get_logger().info(f"Launch Requested. Target altitude: {target_pose.pose.position.z}m")
        self.setpoint_pose = target_pose

        self.master_fsm_transition(FSM.Event.SERVICE_CALL_LAUNCH)
        FSM.service_call_launch_done(True)

        response.success = True
        response.message = "Drone taking off."
        return response


    def callback_test(self, request, response):
        """Handle Test Command: start waypoint finite state machine."""
        if not FSM.state_equal(self.master_fsm, FSM.State.HOVERING):
            response.success = False
            response.message = f"Cannot test from state {self.master_fsm}"
            return response

        if SEARCH_WAYPOINTS is None:
            response.success = False
            response.message = "Waypoints not received."
            return response

        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

        self.fsm_active = True
        self.fsm_waypoint_index = 0
        self.fsm_hold_start_time = None

        # Immediately command the first waypoint so motion starts before next timer tick.
        self.update_waypoint_target(0)
        self.get_logger().info("Test requested, FSM Requested. Starting waypoint FSM.")

        self.master_fsm_transition(FSM.Event.SERVICE_CALL_TEST)
        FSM.service_call_test_done(True)

        response.success = True
        response.message = "FSM started. Following waypoints."
        return response
    

    def callback_land(self, request, response):
        """Handle LAND command: descend back to initial altitude"""        
        self.set_mode("AUTO.LAND")

        response.success = True
        response.message = "Drone is landing."
        return response


    def callback_abort(self, request, response):
        """
        Handle ABORT command: descend back to initial altitude
        CURRENTLY: hands over control to manual mode
        """
        self.callback_land(request, response)
        self.fsm_active = False
        self.fsm_hold_start_time = None

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
        current_pose.pose = msg.pose
        
        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info(f"Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")

        self.latest_pose = current_pose
        if LOG_LATEST_POSE:
            self.get_logger().info(f"Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")

    ############################################################################
    # Waypoints callback
    ############################################################################
    def update_waypoint_target(self, waypoint_index):
        """Set self.setpoint_pose to waypoint at index from SEARCH_WAYPOINTS (shape 3xN)."""
        if SEARCH_WAYPOINTS is None:
            return

        new_waypoint = PoseStamped()
        new_waypoint.header.stamp = self.get_clock().now().to_msg()
        new_waypoint.header.frame_id = "map"
        new_waypoint.pose.position.x = float(SEARCH_WAYPOINTS[0, waypoint_index])
        new_waypoint.pose.position.y = float(SEARCH_WAYPOINTS[1, waypoint_index])
        new_waypoint.pose.position.z = float(SEARCH_WAYPOINTS[2, waypoint_index])

        if self.latest_pose is not None:
            new_waypoint.pose.orientation = self.latest_pose.pose.orientation

        self.setpoint_pose = new_waypoint


    def run_waypoint_fsm(self):
        """Finite state machine to follow waypoints sequentially in a continuous loop."""
        if not self.fsm_active:
            return

        if SEARCH_WAYPOINTS is None or SEARCH_WAYPOINTS.shape[1] == 0 or self.latest_pose is None:
            return

        if self.fsm_waypoint_index >= SEARCH_WAYPOINTS.shape[1]:
            self.fsm_waypoint_index = 0
            self.fsm_hold_start_time = None
            self.get_logger().info("Waypoint FSM loop restart: returning to waypoint 1.")
            return

        # Always command the current waypoint while evaluating progress.
        self.update_waypoint_target(self.fsm_waypoint_index)

        current_pos = np.array([
            self.latest_pose.pose.position.x,
            self.latest_pose.pose.position.y,
            self.latest_pose.pose.position.z,
        ])
        target_pos = SEARCH_WAYPOINTS[:, self.fsm_waypoint_index]
        distance = np.linalg.norm(current_pos - target_pos)

        # Evaluate waypoint hold condition
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if distance <= WAYPOINT_REACH_TOL:
            if self.fsm_hold_start_time is None:
                self.fsm_hold_start_time = now_s
            elif (now_s - self.fsm_hold_start_time) >= WAYPOINT_HOLD_TIME:
                self.get_logger().info(f"Reached waypoint {self.fsm_waypoint_index + 1}/{SEARCH_WAYPOINTS.shape[1]}")
                self.fsm_waypoint_index += 1
                self.fsm_hold_start_time = None
                
                if self.fsm_waypoint_index > SEARCH_WAYPOINTS.shape[1]:
                    self.fsm_waypoint_index = 0
                    self.get_logger().info("Waypoint loop complete: restarting from waypoint 1.")

                self.update_waypoint_target(self.fsm_waypoint_index)
        else:
            self.fsm_hold_start_time = None
            
    ############################################################################
    # Marker functions
    ############################################################################
    def marker_world_pose_callback(self, msg): # PoseStamped
        pass

    ############################################################################
    # Evaluate State
    ############################################################################
    def evaluate_state(self):
        if FSM.state_equal(self.master_fsm, FSM.State.LAUNCHING):
            # if np.linalg.norm(self.setpoint_pose.pose, self.latest_pose.pose)
            if utils.PoseStamped_dist(self.latest_pose, self.setpoint_pose) < WAYPOINT_REACH_TOL:
                self.master_fsm_transition(FSM.Event.REACHED_HOVER_HEIGHT)

        elif FSM.state_equal(self.master_fsm, FSM.State.HOVERING) and FSM.SERVICE_CALL_TEST_DONE:
            if (timer aruco pos not received):
                self.master_fsm_transition(FSM.Event.TIMER_NOT_RECEIVED_ARUCO_POSITION)
            elif (received aruco position):
                self.master_fsm_transition(FSM.Event.RECEIVED_ARUCO_POSITION)

        elif FSM.state_equal(self.master_fsm, FSM.State.SEARCHING):
            if (received aruco position):
                self.master_fsm_transition(FSM.Event.RECEIVED_ARUCO_POSITION)
        
        elif FSM.state_equal(self.master_fsm, FSM.State.TRACKING):
            if utils.PoseStamped_dist(self.latest_pose, self.setpoint_pose) < WAYPOINT_REACH_TOL:
                self.master_fsm_transition(FSM.Event.REACHED_SETPOINT)
            elif (timer aruco pos not received):
                self.master_fsm_transition(FSM.Event.TIMER_NOT_RECEIVED_ARUCO_POSITION)
            elif (received aruco position):
                self.master_fsm_transition(FSM.Event.RECEIVED_ARUCO_POSITION)


    ############################################################################
    # Publisher functions
    ############################################################################
    def publish_setpoint(self):
        self.setpoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint_pose)
        if LOG_SETPOINT:
            self.get_logger().info(f"Published setpoint: x={self.setpoint_pose.pose.position.x}, y={self.setpoint_pose.pose.position.y}, z={self.setpoint_pose.pose.position.z}")


    ############################################################################
    # Print Functions
    ############################################################################
    def print_setpoint(self):
        self.get_logger().info(f"Current setpoint: x={self.setpoint_pose.pose.position.x}, y={self.setpoint_pose.pose.position.y}, z={self.setpoint_pose.pose.position.z}")
        

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
