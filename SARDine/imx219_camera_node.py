import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_system_default


TIMER_30_HZ = 1/30 # [1/Hz]
FRAMERATE = 60



def gstreamer_pipeline(
    capture_w=1280, capture_h=720, display_w=1280, display_h=720, framerate=FRAMERATE, flip_method=0
):
    return (
        f"nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM), width={capture_w}, height={capture_h}, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_w}, height={display_h}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink"
    )


class IMX219CameraNode(Node):
    def __init__(self):
        super().__init__('imx219_camera_node')
        self.publisher = self.create_publisher(
            Image, 
            '/camera/image_raw', 
            qos_profile_system_default
        )
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return

        self.create_timer(TIMER_30_HZ, self.publish_frame)
        self.get_logger().info("IMX219 camera node initialized.")


    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame.")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher.publish(msg)


    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMX219CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
