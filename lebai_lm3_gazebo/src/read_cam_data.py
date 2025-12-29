import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        # 1. Subscribe to the Image Topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Waiting for image data on /camera1/image_raw...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display the image in a window
            cv2.imshow("Camera 1 Feed", cv_image)
            
            # Required to update the window (1ms delay)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()