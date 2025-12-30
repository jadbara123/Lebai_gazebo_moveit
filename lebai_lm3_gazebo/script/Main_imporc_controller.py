import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from lebai_lm3_gazebo.srv import ExecuteWaypoints 

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# OpenCV Imports
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraModule:
    """
    Handles Camera subscriptions and image processing.
    Requires a ROS Node instance to be passed in __init__.
    """
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()
        self.camera_frame_id = None
        self.camera_info_received = False
        self.cv_image = None

        # Subscribe to Image
        self.image_sub = self.node.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to Camera Info
        self.info_sub = self.node.create_subscription(
            CameraInfo,
            '/camera1/camera_info',
            self.info_callback,
            10
        )

    def pixel_to_cam_pose(self, pixels):
        #origin 460, 344
        x0, y0 = 460, 344
        x, y = pixels
        coorx = (x-x0)*0.1/104
        coory = (y-y0)*0.1/104
        return [coorx, coory]
    
    def info_callback(self, msg):
        if not self.camera_info_received:
            # Capture the frame ID from the message header or hardcode as per your setup
            # self.camera_frame_id = msg.header.frame_id 
            self.camera_frame_id = 'cam_kamera_tutucu' 
            self.node.get_logger().info(f"Camera Info Received. Frame ID set to: {self.camera_frame_id}")
            self.camera_info_received = True

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", self.cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")

    def corner_coordinates(self):
        if self.cv_image is not None:
            # 1. Convert BGR image to HSV (better for color segmentation)
            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            # 2. Define range of blue color in HSV
            # Note: These values might need slight tuning based on your lighting
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

            # 3. Create a mask to filter only blue colors
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # 4. Optional: Remove noise (morphological opening)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # 5. Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            all_corners = []

            for cnt in contours:
                # 6. Filter out small noise based on area
                area = cv2.contourArea(cnt)
                if area > 500:  # Adjust threshold as needed
                    # 7. Approximate the contour to a polygon to get corners
                    epsilon = 0.04 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)
                    
                    # Check if it is a rectangle/quadrilateral (4 points)
                    # You can remove this 'if' if you want corners for any blue shape
                    if len(approx) == 4:
                        # Reshape to a simple list of (x, y) points
                        corners = approx.reshape(-1, 2).tolist()
                        all_corners.append(corners)
            print(all_corners)
            return all_corners[0]
        
        return []


class MotionModule:
    """
    Handles Service Requests and TF transformations.
    Requires a ROS Node instance to be passed in __init__.
    """
    def __init__(self, node):
        self.node = node
        
        # --- Service Client Setup ---
        self.cli = self.node.create_client(ExecuteWaypoints, '/execute_waypoints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')
        self.req = ExecuteWaypoints.Request()

        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def transform_camera_to_base(self, coords, z, frame_id):
        if frame_id is None:
            self.node.get_logger().warn("Cannot transform: Frame ID is None")
            return None

        x, y = coords
        # 1. Create a PointStamped
        point_camera = PointStamped()
        point_camera.header.frame_id = frame_id
        point_camera.header.stamp = rclpy.time.Time().to_msg()
        point_camera.point.x = float(x)
        point_camera.point.y = float(y)
        point_camera.point.z = float(z)

        try:
            # 2. Look up the transform
            # We use a timeout duration here to ensure buffer has data
            transform = self.tf_buffer.lookup_transform(
                'base_link',             # Target Frame
                frame_id,                # Source Frame
                rclpy.time.Time(),       # Time (0 = latest)
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 3. Apply the transform
            point_base = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            self.node.get_logger().info(
                f"TRANSFORM SUCCESS: Cam({x}, {y}, {z}) -> Base({point_base.point.x:.3f}, {point_base.point.y:.3f}, {point_base.point.z:.3f})"
            )
            return point_base.point

        except TransformException as ex:
            self.node.get_logger().error(f'Could not transform point: {ex}')
            return None

    def request_point(self, point = None, rot=None):
        p1 = Pose()
        if point is None:
            p1.position.x = -0.48
            p1.position.y = -0.08
            p1.position.z = 0.32
        else:
            p1.position.x = point.x
            p1.position.y = point.y
            p1.position.z = point.z

        if rot is None:
            p1.orientation.w = 0.3805
            p1.orientation.x = 0.8163
            p1.orientation.y = -0.4345
            p1.orientation.z = -0.0042
        else:
            p1.orientation = rot
        
        self.req.poses = [p1]
        
        # Call async using the node attached to this class
        future = self.cli.call_async(self.req)
        
        # Note: We cannot use spin_until_future_complete(self, ...) here because 
        # 'self' is not a Node. We must return the future or handle it differently.
        # However, since we are calling this from main() which has the loop, 
        # we will return the future to be handled there.
        return future
    
    

class RobotController(Node):
    """
    Main ROS 2 Node that owns the Camera and Motion modules.
    """
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Initialize the helper classes, passing 'self' (the node) to them
        self.cam = CameraModule(self)
        self.motion = MotionModule(self)
        

def main(args=None):
    rclpy.init(args=args)
    
    # Create the single main node
    robot_node = RobotController()

    print("Waiting for camera info to initialize TF...")
    
    # 1. Spin specifically to wait for Camera Info (to get Frame ID)
    while rclpy.ok() and not robot_node.cam.camera_info_received:
        rclpy.spin_once(robot_node)

    # 2. Spin a few more times to fill TF buffer
    for _ in range(10): 
        rclpy.spin_once(robot_node)

    # 3. Calculate Transform
    # We access the frame_id stored in the camera module
    frame_id = robot_node.cam.camera_frame_id
    print("Sending Movement Request...")
    future = robot_node.motion.request_point()
    
    # Wait for the movement service to complete
    rclpy.spin_until_future_complete(robot_node, future)
    corners = robot_node.cam.corner_coordinates()
    print(corners)
    corners.append(corners[0])

    for i in corners:
        pose = robot_node.cam.pixel_to_cam_pose(i)
        target_point_base = robot_node.motion.transform_camera_to_base(pose, 0.40, frame_id)
    
        # 4. Request Movement
        if target_point_base:
            print("Sending Movement Request...")
            future = robot_node.motion.request_point(target_point_base)
            
            # Wait for the movement service to complete
            rclpy.spin_until_future_complete(robot_node, future)
            print("Movement Finished (or Service returned).")
    # Transform point relative to camera (-0.1, 0.0, 0.40)
    """target_point_base = robot_node.motion.transform_camera_to_base([-0.1, 0.0], 0.40, frame_id)
    
    # 4. Request Movement
    if target_point_base:
        print("Sending Movement Request...")
        future = robot_node.motion.request_point(target_point_base)
        
        # Wait for the movement service to complete
        rclpy.spin_until_future_complete(robot_node, future)
        print("Movement Finished (or Service returned).")"""

    try:
        # 5. Continuous Spin for Camera Feed
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()