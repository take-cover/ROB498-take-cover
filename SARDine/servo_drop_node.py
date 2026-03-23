#!/usr/bin/env python3
"""
ROS2 node: servo_drop_node  (Jetson Nano)
Subscribes to /rob498_drone_1/servo/drop (std_msgs/Bool).
When data=True,  drives an SG90 servo to the DROP position.
When data=False, returns to the HOME position.

Hardware assumptions
--------------------
- Jetson Nano with Jetson.GPIO installed
  (pip install Jetson.GPIO  or  sudo apt install python3-jetson-gpio)
- SG90 signal wire → pin defined by SERVO_PIN (BOARD numbering by default)
- SG90 PWM: 50 Hz
    1.0 ms pulse → 0°   (dc =  5.0 %)
    1.5 ms pulse → 90°  (dc =  7.5 %)
    2.0 ms pulse → 180° (dc = 10.0 %)
  Adjust HOME_DC / DROP_DC to match your mechanism.

Jetson Nano PWM-capable header pins (BOARD numbering)
------------------------------------------------------
  Pin 32  →  GPIO12  (PWM0)   ← recommended, hardware PWM
  Pin 33  →  GPIO13  (PWM2)   ← alternative, hardware PWM
  Software PWM works on any GPIO pin but may jitter under load.

Permissions
-----------
  sudo usermod -aG gpio $USER   # allow GPIO access without sudo
  (then log out and back in)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ── Hardware config ────────────────────────────────────────────────────────────
SERVO_PIN  = 32          # BOARD pin number (physical header pin)
PWM_FREQ   = 50          # Hz — standard for SG90

# Duty cycle = (pulse_ms / 20 ms) * 100
HOME_DC  = 7.5           # ~1.5 ms → 90°  (stowed position)
DROP_DC  = 10.0          # ~2.0 ms → 180° (release position) — tune as needed
# ──────────────────────────────────────────────────────────────────────────────

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
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            self._pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
            self._pwm.start(HOME_DC)
            self.get_logger().info(
                f'Servo initialised on BOARD pin {SERVO_PIN} @ {PWM_FREQ} Hz. '
                f'Home DC={HOME_DC}%'
            )
        else:
            self._pwm = None
            self.get_logger().warn(
                'Jetson.GPIO not available — running in simulation mode. '
                'Install with:  pip install Jetson.GPIO'
            )

        self._dropped = False   # latch: avoid re-sending the same command

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Bool,
            '/rob498_drone_1/servo/drop',
            self._drop_callback,
            10
        )
        self.get_logger().info(
            'Subscribed to /rob498_drone_1/servo/drop  [std_msgs/Bool]'
        )

    # ── Callback ─────────────────────────────────────────────────────────────
    def _drop_callback(self, msg: Bool):
        if msg.data and not self._dropped:
            self.get_logger().info('DROP → moving servo to drop position.')
            self._set_duty_cycle(DROP_DC)
            self._dropped = True

        elif not msg.data and self._dropped:
            self.get_logger().info('RESET → returning servo to home position.')
            self._set_duty_cycle(HOME_DC)
            self._dropped = False

    def _set_duty_cycle(self, dc: float):
        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(dc)
        else:
            self.get_logger().info(f'[SIM] Duty cycle → {dc}%')

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        if HAS_GPIO and self._pwm is not None:
            self._pwm.stop()
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
    