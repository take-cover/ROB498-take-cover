#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rclpy.qos import qos_profile_system_default

SERVO_ASSERT_PIN  = 12          # BOARD pin number (physical header pin)

try:
    import Jetson.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class ServoDropNode(Node):
    def __init__(self):
        super().__init__('servo_drop_node')

        # ── Servo setup ──────────────────────────────────────────────────────
        if HAS_GPIO:
            GPIO.setmode(GPIO.BOARD)          # use physical pin numbers
            GPIO.setup(SERVO_ASSERT_PIN, GPIO.OUT)
        else:
            self.get_logger().warn(
                'Jetson.GPIO not available — running in simulation mode. '
                'Install with:  pip install Jetson.GPIO'
            )

        self._dropped = False   # latch: avoid re-sending the same command

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Bool,
            '/take_cover/servo/drop',
            self._drop_callback,
            qos_profile_system_default
        )
        self.get_logger().info(
            'Subscribed to /take_cover/servo/drop  [std_msgs/Bool]'
        )

    # ── Callback ─────────────────────────────────────────────────────────────
    def _drop_callback(self, msg: Bool):
        if msg.data and not self._dropped:
            self.get_logger().info('DROP → moving servo to drop position.')
            GPIO.output(SERVO_ASSERT_PIN, GPIO.HIGH)
            self._dropped = True

        elif not msg.data and self._dropped:
            self.get_logger().info('RESET → returning servo to home position.')
            GPIO.output(SERVO_ASSERT_PIN, GPIO.LOW)
            self._dropped = False

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        if HAS_GPIO:
            GPIO.cleanup()
            self.get_logger().info('GPIO cleaned up.')
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ServoDropNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    