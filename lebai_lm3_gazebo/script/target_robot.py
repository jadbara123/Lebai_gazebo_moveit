import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from lebai_lm3_gazebo.srv import ExecuteWaypoints 

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs # IMORTANT: Imports the hook for do_transform_point


# at 0.4 distance from cam -0.1 in y axis is equal to -104 pixel and the middle is 460, 344 
# and -0.1 in x is equal to 104 pixels to the left
# OpenCV and Bridge imports
import cv2
from cv_bridge import CvBridge

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        
        # --- Service Client Setup ---
        self.cli = self.create_client(ExecuteWaypoints, '/execute_waypoints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = ExecuteWaypoints.Request()

        # --- TF2 Setup ---
        # The buffer stores a history of transforms (usually 10 seconds by default)
        self.tf_buffer = Buffer()
        # The listener subscribes to /tf and fills the buffer automatically
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Variable to store the specific frame name of the camera (e.g., "camera_link_optical")
        self.camera_frame_id = None 

        # --- Camera Setup ---
        self.bridge = CvBridge()
        
        # Subscribe to Image
        self.image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to Camera Info
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera1/camera_info',
            self.info_callback,
            10
        )
        
        self.camera_info_received = False

    def info_callback(self, msg):
        """Reads camera calibration data and captures frame ID."""
        if not self.camera_info_received:
            # Capture the frame ID from the message header
            self.camera_frame_id = 'cam_kamera_tutucu'
            self.get_logger().info(f"Camera Info Received. Frame ID: {self.camera_frame_id}")
            self.camera_info_received = True

    def image_callback(self, msg):
        """Reads raw image data and displays it using OpenCV."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def pixel_to_cam_pose(self, pixels):
        #origin 460, 344
        x0, y0 = 460, 344
        x, y = pixels
        coorx = (x-x0)*0.1/104
        coory = (y-y0)*0.1/104
        return coorx, coory
        

    def transform_camera_to_base(self, x, y, z):
        if self.camera_frame_id is None:
            self.get_logger().warn("Cannot transform: Camera Frame ID not yet received.")
            return None

        # 1. Create a PointStamped (Point + Header)
        point_camera = PointStamped()
        point_camera.header.frame_id = self.camera_frame_id
        # "Time()" with no args gets the current time (0), asking for the latest available transform
        point_camera.header.stamp = rclpy.time.Time().to_msg()
        point_camera.point.x = float(x)
        point_camera.point.y = float(y)
        point_camera.point.z = float(z)

        try:
            # 2. Look up the transform from Camera -> Base Link
            transform = self.tf_buffer.lookup_transform(
                'base_link',             # Target Frame
                self.camera_frame_id,    # Source Frame
                rclpy.time.Time()        # Time
            )

            # 3. Apply the transform
            point_base = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            self.get_logger().info(
                f"TRANSFORM SUCCESS: Cam({x}, {y}, {z}) -> Base({point_base.point.x:.3f}, {point_base.point.y:.3f}, {point_base.point.z:.3f})"
            )
            return point_base.point

        except TransformException as ex:
            self.get_logger().error(f'Could not transform point: {ex}')
            return None

    def middle_pose(self):
        p1 = Pose()
        p1.position.x = -0.15
        p1.position.y = -0.43
        p1.position.z = 0.30
        p1.orientation.w = 0.0
        p1.orientation.x = 0.7071068
        p1.orientation.y = 0.7071068
        p1.orientation.z = 0.0
        
        self.req.poses = [p1]
        print(f"Sending Request: Middle Pose")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def clear_pose(self):
        p1 = Pose()
        p1.position.x = -0.48
        p1.position.y = -0.08
        p1.position.z = 0.32
        p1.orientation.w = 0.3805
        p1.orientation.x = 0.8163
        p1.orientation.y = -0.4345
        p1.orientation.z = -0.0042
        
        self.req.poses = [p1]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def request_point(self, point, rot = None):
        p1 = Pose()
        p1.position.x = point.x
        p1.position.y = point.y
        p1.position.z = point.z

        if rot is None:
            p1.orientation.w = 0.0
            p1.orientation.x = 0.7071068
            p1.orientation.y = 0.7071068
            p1.orientation.z = 0.0
        else:
            p1.orientation.w = rot.w
            p1.orientation.x = rot.x
            p1.orientation.y = rot.y
            p1.orientation.z = rot.z
        
        self.req.poses = [p1]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = WaypointClient()

    # --- 1. Wait for Camera Info to get the Frame ID ---
    print("Waiting for camera info to initialize TF...")
    while rclpy.ok() and not client.camera_info_received:
        rclpy.spin_once(client)
    
    for _ in range(5): rclpy.spin_once(client) 
    transformed_point = client.transform_camera_to_base(-0.0, 0.0, 0.50)
    
    client.clear_pose()

    print("Keeping node alive for camera feed. Press Ctrl+C to exit.")
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()