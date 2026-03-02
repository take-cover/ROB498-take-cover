import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from tf_transformations import quaternion_multiply, quaternion_from_euler

import numpy as np
import math


G_HEIGHT = 0.4
FREQ_30_HZ = 1/30
FREQ_0_5_HZ = 2

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1')
        # Poses
        self.initial_pose = None # Startup pose (first power on)
        self.latest_pose = None

        self.waypoint_pose = PoseStamped() # Pose to hold during test
        self.test_state = 0 # 0 = not started, 1 = hold position during test
        
        self.vicon_enabled = False
        self.state = State()

        # Set up publishers
        self.ego_pos_pub = self.create_publisher(
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

        # I believe flight controller compares waypoint to current position

        # Set up subscribers
        self.vicon_sub = self.create_subscription(
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

        # Set up mavros clients
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # Create services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)

        self.get_logger().info("CommNode initiailized; initial pose not yet received.")


    # ----------------------------- helper functions -------------------------------------------------
    def update_waypoint_pose(self, PoseStamped):
        self.waypoint_pose = PoseStamped
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pose.header.frame_id = "map"

    # ---------------------------- commands -------------------------------------------------
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


    # --------------------------- Service callbacks ------------------------------------------------------
    def callback_launch(self, request, response):
        """Handle LAUNCH command: take off to desired height above initial pose"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response
        
        self.set_mode("ALTCTL")
        self.get_logger().info("Initiate manual takeoff.")
       
        # if self.state.mode != "OFFBOARD":
        #     self.set_mode("OFFBOARD")

        # # arm drone if not armed yet
        # if not self.state.armed:
        #     print("Arming drone...")
        #     self.arm_drone(True)

        # target_z = self.latest_pose.pose.position.z + G_HEIGHT

        # self.get_logger().info(f"Launch Requested. Target altitude: {G_HEIGHT}m")

        response.success = True
        response.message = "Drone taking off."

        return response

    def callback_test(self, request, response):
        """Handle TEST command: Set waypoint & wait until TA done collecting data..."""
        self.get_logger().info("Test Requested. Starting test sequence.")

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            
        if self.test_state == 0:
            self.update_waypoint_pose(self.latest_pose)
            self.test_state = 1 # set to fixed position (position at the start of test call)

        response.success = True
        response.message = "Test has started. Recording data."
        return response
    
    def callback_land(self, request, response):
        """Handle LAND command: descend back to initial altitude"""
        # if self.initial_pose is None:
        #     response.success = False
        #     response.message = "No initial pose."
        #     return response

        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.pose.position.z}m")
        
        land_pose = PoseStamped()
        land_pose.header.stamp = self.get_clock().now().to_msg()
        land_pose.header.frame_id = "map"
        land_pose.pose = self.latest_pose.pose
        land_pose.pose.position.z = self.initial_pose.pose.position.z - 0.05 # set waypoint to just below initial position to encourage landing

        self.update_waypoint_pose(land_pose)

        response.success = True
        response.message = "Drone is landing."
        return response

    def callback_abort(self, request, response):
        """
        Handle ABORT command: descend back to intiial altitude
        CURRENTLY: hands over control to manual mode
        """

        self.get_logger().info(f"ABORT Requested! Returning control to manual")
        self.set_mode("ALTCTL")

        response.success = True
        response.message = "Drone is landing."
        return response


    def realsense_callback(self, msg):
        """Update pose from RealSense"""    
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose

        # Rotate pose 90 deg about z axis to align with FC
        q_orig = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        q_rot = quaternion_from_euler(0, 0, math.pi/2)
        q_new = quaternion_multiply(q_orig, q_rot)

        current_pose.pose.pose.orientation = Quaternion(
            x=q_new[0],
            y=q_new[1],
            z=q_new[2],
            w=q_new[3]
        )

        # Update pose(s)
        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.get_logger().info(f"Realsense - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")
            
        self.latest_pose = current_pose
        
        self.get_logger().info(f"Realsense - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")
    

    def vicon_callback(self, msg):
        """Update pose from Vicon"""
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.get_logger().info(f"Vicon - Set initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}, z={self.initial_pose.pose.position.z}")
        
        self.latest_pose = msg.pose
        self.get_logger().info(f"Vicon - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")


    def publish_position(self):
        """Publishes a new desired position."""
        if self.latest_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_pos_pub.publish(self.latest_pose)
            self.get_logger().info(f"Published latest self pose")


    def publish_waypoint(self):
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint_pose)
        self.get_logger().info(f"Published waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")


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
