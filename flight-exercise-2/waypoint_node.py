#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, qos_profile_system_default
import numpy as np
import time

def quaternion_multiply(q1, q2):
    """Perform quaternion multiplication."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

class DroneCommNode(Node):
    def __init__(self):
        super().__init__("drone_comm")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        # Store latest pose (default to None)
        self.initial_pose = None
        self.latest_pose = None
        self.source = None  # 'vicon' or 'realsense'
        
        # Create service servers
        self.srv_launch = self.create_service(Trigger, "rob498_drone_3/comm/launch", self.handle_launch)
        self.srv_land = self.create_service(Trigger, "rob498_drone_3/comm/land", self.handle_land)
        self.srv_abort = self.create_service(Trigger, "rob498_drone_3/comm/abort", self.handle_abort)
        self.srv_test = self.create_service(Trigger, "rob498_drone_3/comm/test", self.handle_test)

        # while not self.set_mode_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().warn('Waiting for set_mode service...')
        
        self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_system_default)

        # VICON Subscriber
        self.vicon_sub = self.create_subscription(
            PoseStamped,
            "/vicon/ROB498_Drone/ROB498_Drone",
            self.vicon_callback,
            qos_profile_system_default
        )

        # RealSense Subscriber
        self.realsense_sub = self.create_subscription(
            Odometry,
            "/camera/pose/sample",
            self.realsense_callback,
            qos_profile_system_default
        )

        # vision publisher
        self.ego_pub = self.create_publisher(PoseStamped,'/mavros/vision_pose/pose', qos_profile_system_default)
        # Timer to publish vision pose at 20 Hz
        self.create_timer(1/20, self.publish_vision_pose)

        # Publisher for MAVROS setpoints
        self.pose_publisher = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", qos_profile_system_default)
        # MAVROS clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.state = State()
        self.timer = self.create_timer(0.02, self.cmdloop_callback) # set arm and offboard mode if haven't
        
        if self.latest_pose:
            self.hover_pose = self.latest_pose
        else:
            self.hover_pose = PoseStamped()
        self.update_hover_pose(0.0, 0.0, 0.0)

        # Timer to publish waypoints at 20 Hz
        self.create_timer(1/20, self.publish_waypoint)

        self.launch = False
        self.calibrate_vicon_z = None
        self.calibrate_rs_z = None

        self.get_logger().info("Drone communication node started. Listening to VICON and RealSense.")

    def state_cb(self, msg):
        self.state = msg
        self.get_logger().info(f"Current mode: {self.state.mode}")
    
    def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)
    
    def arm(self, arm_status):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = arm_status
            self.arming_client.call_async(req)
            
    def cmdloop_callback(self):
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        if not self.state.armed:
            self.arm(True)
        
    def vicon_callback(self, msg):
        """Update pose from VICON."""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        if not self.initial_pose:
            self.initial_pose = msg
        self.latest_pose = msg
        self.source = "vicon"

        if self.launch and not self.calibrate_vicon_z:
            self.calibrate_vicon_z = msg.pose.position.z
        # self.vicon_pub.publish(msg)
        # self.get_logger().info(f"VICON Pose Received: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

    def realsense_callback(self, msg):
        """Update pose from RealSense with 90-degree yaw rotation."""
        if self.launch and not self.calibrate_rs_z:
            self.calibrate_rs_z = msg.pose.pose.orientation.z
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # Convert Odometry to PoseStamped
        current_pose_d = PoseStamped()
        current_pose_d.header.stamp = self.get_clock().now().to_msg()
        current_pose_d.header.frame_id = "map"
        current_pose_d.pose = msg.pose.pose

        # Original orientation
        q_orig = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        self.get_logger().info(f"Cam Original: x={msg.pose.pose.orientation.x}, y={msg.pose.pose.orientation.y}, z={msg.pose.pose.orientation.z}")
        # 90-degree yaw quaternion (0, 0, sin(π/4), cos(π/4))
        # q_yaw = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])

        # Apply rotation
        q_new = q_orig #quaternion_multiply(q_yaw, q_orig)

        # Update orientation
        current_pose_d.pose.orientation = Quaternion(
            w=float(q_new[0]),
            x=float(q_new[1]),
            y=float(q_new[2]),
            z=float(q_new[3])
        )
        # self.get_logger().info(f"Cam Converted: x={current_pose_d.pose.orientation.x}, y={current_pose_d.pose.orientation.y}, z={current_pose_d.pose.orientation.z}")

        if not self.initial_pose:
            self.initial_pose = current_pose_d
        self.latest_pose = current_pose_d
        self.source = "realsense"


    def update_hover_pose(self, x, y, z):
        self.hover_pose.header.stamp = self.get_clock().now().to_msg()
        self.hover_pose.header.frame_id = "map"
        self.hover_pose.pose.position.x = x
        self.hover_pose.pose.position.y = y
        self.hover_pose.pose.position.z = z

     # publish to mavros, update latest ego pose from either vicon or realsense
    def publish_vision_pose(self):
        if self.latest_pose:
            # hover_pose_d = PoseStamped()
            hover_pose_d = self.latest_pose
            hover_pose_d.header.stamp = self.get_clock().now().to_msg()
            hover_pose_d.header.frame_id = "map"
            
            self.get_logger().info(f"Latest pose: x={hover_pose_d.pose.position.x}, y={hover_pose_d.pose.position.y}, z={hover_pose_d.pose.position.z}")

            self.ego_pub.publish(hover_pose_d)
            self.get_logger().info(f"Published itself's pose from {self.source}.")
        else:
            self.get_logger().info(f"Published itself's pose from nowhere!")
            
    def publish_waypoint(self):
        """Continuously publish the latest pose to MAVROS at 20 Hz, forcing a hover height."""
        # mode_req = SetMode.Request()
        self.hover_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher.publish(self.hover_pose)
        self.get_logger().info(f"Test cmd Received: x={self.hover_pose.pose.position.x}, y={self.hover_pose.pose.position.y}, z={self.hover_pose.pose.position.z}")
        # self.get_logger().info(f"Published hover waypoint at (0,0,0) from {self.source}, in {mode_req.custom_mode}")

    def handle_launch(self, request, response):
        self.get_logger().info("Launch command received. Taking off...")

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        if not self.state.armed:
            self.arm(True)
        # Change the altitude
        self.hover_pose.header.stamp = self.get_clock().now().to_msg()
        # for testing purpose, set it to 0.5 for now.
        self.hover_pose.pose.position.z = 0.9   # Force drone to hover at 1.5 meters   
        #if self.source == 'realsense':
            #self.hover_pose.pose.position.z = 1.5
        
        # Arm the drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        self.get_logger().info("Launch request sent.")
        # Ensure a response is returned
        response.success = True
        response.message = "Takeoff initiated."
        self.launch = True
        return response
 
    def handle_land(self, request, response):
        self.get_logger().info("Land command received. Trying to land...")

        mode_req = SetMode.Request()
        mode_req.custom_mode = "AUTO.LAND"
        future = self.set_mode_client.call_async(mode_req)
        self.hover_pose.header.stamp = self.get_clock().now().to_msg()
        self.hover_pose.pose.position.z = 0.1
        self.get_logger().info("Landing mode request sent.")
        response.success = True
        return response # Trigger.Response(success=True, message="Landing initiated.")


    def handle_abort(self, request, response):
        self.get_logger().info("Abort command received! Stopping flight.")

        mode_req = SetMode.Request()
        mode_req.custom_mode = "AUTO.RTL"
        self.set_mode_client.call_async(mode_req)
        self.get_logger().info("Return-to-launch (RTL) mode request sent.")

        return response # Trigger.Response(success=True, message="Abort initiated.")


    def handle_test(self, request, response):
        self.hover_pose.header.stamp = self.get_clock().now().to_msg()
        # test is not used for task 2
        offset = self.calibrate_vicon_z - self.calibrate_rs_z
        if offset > 1:
            offset = 1 # CAPPing
        elif offset < -1:
            offset = -1
        self.get_logger().info(f"Adding an offset of {offset}.")

        self.hover_pose.pose.position.z = 1.5 + (offset)  # Force drone to hover at 1.5 meters   
        return response # Trigger.Response(success=True, message="Test acknowledged.")


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommNode()
    rclpy.spin(node)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()