#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Saver(Node):
    def __init__(self):
        super().__init__('saver')
        self.bridge = CvBridge()
        self.window_name = 'camera/image_raw'
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def cb(self, msg):
        # Old one-shot save callback code:
        # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.imwrite('frame.png', img)
        # print("Saved frame.png")
        # rclpy.shutdown()

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Saver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()