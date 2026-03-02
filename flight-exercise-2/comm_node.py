import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import numpy as np


G_HEIGHT = 0.4

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1')
        # Poses
        self.initial_pose = None
        self.initial_pose_cov = None
        self.latest_pose = None
        self.latest_pose_cov = None

        self.waypoint_pose = PoseStamped() # Pose to hold during test
        self.waypoint_state = 0
        # 0 - set to latest pose
        # 1 - set to fixed position
        
        self.vicon_enabled = False

        self.state = State()
        # mavros clients
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        self.get_logger().info("CommNode initiailized; initial pose not yet received.")

        # Set up publishers
        self.ego_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.ego_pos_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/mavros/vision_pose/pose_cov",
            qos_profile_system_default
        )
        self.create_timer(1/30, self.publish_position) # publish vision pose at 30Hz

        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            qos_profile_system_default
        )
        self.create_timer(1/30, self.publish_waypoint) # publish waypoint at 30Hz

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

        # Create services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)


    # ----------------------------- helper functions -------------------------------------------------
    def update_waypoint_pose(self, PoseStamped):
        self.waypoint_pose = PoseStamped
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pose.header.frame_id = "map"

    # ---------------------------- commands -------------------------------------------------
    def cmd_arm_drone(self, arm_status):
        if self.arming_client.service_is_ready():
            print("arming drone")
            req = CommandBool.Request()
            req.value = arm_status
            self.arming_client.call_async(req)
        else:
            print("client service not ready")

    def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)

    # --------------------------- callbacks ------------------------------------------------------
    def callback_launch(self, request, response):
        """Handle LAUNCH command: take off to desired height above initial pose"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response
       
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

        # arm drone if not armed yet
        if not self.state.armed:
            print("calling cmd_arm_drone")
            self.cmd_arm_drone(True)

        target_z = self.latest_pose.pose.position.z + G_HEIGHT

        self.get_logger().info(f"Launch Requested. Target altitude: {G_HEIGHT}m")

        # set hover pose to target altitude above initial position
        if self.waypoint_state == 0:
            self.update_waypoint_pose(self.waypoint_pose.pose.position.x, self.waypoint_pose.pose.position.y, target_z)     
            self.waypoint_state = 1 # set to fixed position

        response.success = True
        response.message = "Drone taking off."

        return response

    def callback_test(self, request, response):
        """Handle TEST command: just wait until TA done collecting data..."""
        self.get_logger().info("Test Requested. Starting test sequence.")

        self.update_waypoint_pose(self.latest_pose.pose.position.x, self.latest_pose.pose.position.y, self.latest_pose.pose.position.z) # maintain current position during test

        response.success = True
        response.message = "Test has started. Recording data."
        return response
    
    def callback_land(self, request, response):
        """Handle LAND command: descend back to intial altitude"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response

        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        self.update_waypoint_pose(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z-0.05) # set waypoint to just below initial position to encourage landing

        response.success = True
        response.message = "Drone is landing."
        return response

    def callback_abort(self, request, response):
        # """Handle ABORT command"""
        # self.get_logger().warn("ABORT Requested! Cutting all thrust.")
        # if self.initial_pose is None:
        #     response.success = False
        #     response.message = "Initial pose not received yet."
        #     return response

        # self.publish_position(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
        # self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        # response.success = True

        # response.message = "Emergency shutdown command sent. Drone should land immediately."
        # return response

        """Handle LAND command: descend back to intial altitude"""
        if self.initial_pose is None:
            response.success = False
            response.message = "No initial pose."
            return response

        self.get_logger().info(f"Landing Requested. Returning to z={self.initial_pose.position.z}m")

        self.update_waypoint_pose(self.latest_pose.position.x, self.latest_pose.position.y, self.latest_pose.position.z-G_HEIGHT) # set waypoint to just below initial position to encourage landing

        response.success = True
        response.message = "Drone is landing."
        return response


    def realsense_callback(self, msg):
        """Update pose from RealSense""" 
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "map"
        current_pose.pose = msg.pose.pose

        current_pose_cov = PoseWithCovarianceStamped()
        current_pose_cov.header.stamp = self.get_clock().now().to_msg()
        current_pose_cov.header.frame_id = "map"
        current_pose_cov.pose = msg.pose

        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.initial_pose_cov = current_pose_cov
            self.get_logger().info(f"Realsense - Set initial pose: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}, z={self.initial_pose.position.z}")
            
        self.latest_pose = current_pose
        self.latest_pose_cov = current_pose_cov
        
        self.get_logger().info(f"Realsense - Latest pose: x={self.latest_pose.pose.position.x}, y={self.latest_pose.pose.position.y}, z={self.latest_pose.pose.position.z}")
    

    def vicon_callback(self, msg):
        """Update pose from Vicon"""
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.get_logger().info(f"Vicon - Set initial pose: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}, z={self.initial_pose.position.z}")
        
        self.latest_pose = msg.pose
        self.get_logger().info(f"Vicon - Latest pose: x={self.latest_pose.position.x}, y={self.latest_pose.position.y}, z={self.latest_pose.position.z}")


    def publish_position(self):
        """Publishes a new desired position."""
        if self.latest_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_pos_pub.publish(self.latest_pose)
            self.get_logger().info(f"Published self pose")

            # pos_cov_msg = self.latest_pose_cov
            # pos_cov_msg.header.stamp = self.get_clock().now().to_msg()
            # pos_cov_msg.header.frame_id = "odom"
            # self.ego_pos_cov_pub.publish(pos_cov_msg)
            # self.get_logger().info(f"Published self pose cov")

            if self.waypoint_state == 0:
                self.update_waypoint_pose(self.latest_pose)

    def publish_waypoint(self):
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint_pose)
        self.get_logger().info(f"Published waypoint: x={self.waypoint_pose.pose.position.x}, y={self.waypoint_pose.pose.position.y}, z={self.waypoint_pose.pose.position.z}")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
