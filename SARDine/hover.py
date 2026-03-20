import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_srvs.srv import Trigger

class ArucoHoverCommander(Node):
    def __init__(self):
        super().__init__('aruco_hover_commander')

        self.waypoint_sent = False

        self.aruco_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose',
            self.aruco_callback,
            10
        )

        self.waypoint_pub = self.create_publisher(
            PoseArray,
            '/rob498_drone_1/comm/waypoints',
            10
        )

        self.test_client = self.create_client(Trigger, 'rob498_drone_1/comm/test')

        self.get_logger().info("Aruco Hover Commander initialized. Searching for target...")

    def aruco_callback(self, msg):
        if self.waypoint_sent:
            return

        self.get_logger().info("ArUco tag detected! Calculating hover waypoint...")
        target_pose = Pose()

        # determine waypoint based on detected ArUco tag position
        # double check frame of reference and adjust if necessary
        target_pose.position.x = msg.pose.position.x
        target_pose.position.y = msg.pose.position.y
        target_pose.position.z = msg.pose.position.z + 5.0  
        target_pose.orientation.w = 1.0
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0

        waypoint_msg = PoseArray()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.poses = [target_pose]

        # publish waypoint
        self.waypoint_pub.publish(waypoint_msg)
        self.get_logger().info(f"Published waypoint: x={target_pose.position.x:.2f}, y={target_pose.position.y:.2f}, z={target_pose.position.z:.2f}")

        self.waypoint_sent = True

        self.trigger_fsm()

    def trigger_fsm(self):
        self.get_logger().info("Waiting for FSM /test service to become available...")
        self.test_client.wait_for_service()

        self.get_logger().info("Triggering drone flight...")
        req = Trigger.Request()
        future = self.test_client.call_async(req)
        future.add_done_callback(self.test_service_callback)

    def test_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"FSM Trigger Response: {response.success} - {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoHoverCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()