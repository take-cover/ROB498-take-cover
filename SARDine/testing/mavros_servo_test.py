import math
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong

class ServoTest(Node):
    def __init__(self):
        super().__init__('servo_test')
        self.cli = self.create_client(CommandLong, '/mavros/cmd/command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /mavros/cmd/command...')

        for value in [0.0, 0.3, -0.3, 0.0]:
            self.send_actuator(value)
            time.sleep(2.0)

    def send_actuator(self, value):
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 187      # MAV_CMD_DO_SET_ACTUATOR
        req.confirmation = 0
        req.param1 = float(value)  # MAIN 8 mapped to Offboard Actuator Set 1
        req.param2 = math.nan
        req.param3 = math.nan
        req.param4 = math.nan
        req.param5 = math.nan
        req.param6 = math.nan
        req.param7 = math.nan

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        print(type(future))
        print(future.result())
        if future.result() is not None:
            self.get_logger().info(
                f'value={value}, success={future.result().success}, result={future.result().result}'
            )
        else:
            self.get_logger().error(f'No ACK for value={value}')

def main():
    rclpy.init()
    node = ServoTest()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()