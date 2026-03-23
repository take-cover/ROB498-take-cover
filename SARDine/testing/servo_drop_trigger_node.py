#!/usr/bin/env python3
"""
ROS2 node: servo_drop_trigger_node
Publishes to /rob498_drone_1/servo/drop (std_msgs/Bool).

Exposes a service /rob498_drone_1/servo/drop/trigger (std_srvs/SetBool)
so you can command the servo from the CLI:

  # Drop:
  ros2 service call /rob498_drone_1/servo/drop/trigger std_srvs/srv/SetBool "data: true"

  # Reset:
  ros2 service call /rob498_drone_1/servo/drop/trigger std_srvs/srv/SetBool "data: false"

The node continuously re-publishes the current state at a fixed rate so the
subscriber always has a live value (useful if it starts after this node).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

TOPIC   = '/rob498_drone_1/servo/drop'
SERVICE = '/rob498_drone_1/servo/drop/trigger'
PUBLISH_RATE_HZ = 10.0   # re-publish current state at this rate


class ServoDropTriggerNode(Node):
    def __init__(self):
        super().__init__('servo_drop_trigger_node')

        self._current_state = False   # False = home, True = drop

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Bool, TOPIC, 10)

        # ── Service ──────────────────────────────────────────────────────────
        self._srv = self.create_service(SetBool, SERVICE, self._trigger_callback)

        # ── Heartbeat timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_state)

        self.get_logger().info(f'Publishing  → {TOPIC}  [std_msgs/Bool]')
        self.get_logger().info(f'Service     → {SERVICE}  [std_srvs/SetBool]')
        self.get_logger().info('')
        self.get_logger().info('Trigger from CLI:')
        self.get_logger().info(
            f'  DROP:  ros2 service call {SERVICE} std_srvs/srv/SetBool "data: true"'
        )
        self.get_logger().info(
            f'  RESET: ros2 service call {SERVICE} std_srvs/srv/SetBool "data: false"'
        )

    # ── Service callback ─────────────────────────────────────────────────────
    def _trigger_callback(self, request: SetBool.Request, response: SetBool.Response):
        self._current_state = request.data
        self._publish_state()   # publish immediately, don't wait for timer

        action = 'DROP' if request.data else 'RESET'
        self.get_logger().info(f'Service call received → {action}')

        response.success = True
        response.message = f'Published {action} (data={request.data})'
        return response

    # ── Periodic publish ─────────────────────────────────────────────────────
    def _publish_state(self):
        msg = Bool()
        msg.data = self._current_state
        self._pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ServoDropTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()