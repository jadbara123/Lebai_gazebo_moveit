import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState  # <--- NEW IMPORT

class EndEffectorPoseListener(Node):

    def __init__(self):
        super().__init__('ee_pose_listener')
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Joint State Setup (NEW) ---
        self.latest_joint_msg = None
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states', # Standard topic name
            self.joint_state_callback,
            10
        )

        # Create a timer to print the data every 0.5 seconds
        self.timer = self.create_timer(0.5, self.print_status)

    def joint_state_callback(self, msg):
        """Callback to constantly update the latest joint data."""
        self.latest_joint_msg = msg

    def print_status(self):
        from_frame_rel = 'base_link'
        to_frame_rel = 'g_brush_link'

        print(f"\n--- Robot Status ---")

        # 1. GET & PRINT POSE
        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())

            p = t.transform.translation
            x, y, z = p.x, p.y, p.z

            r = t.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w

            print(f"EE Position (XYZ): [{x:.4f}, {y:.4f}, {z:.4f}]")
            print(f"EE Rotation (Quat): [{qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}]")

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')

        # 2. PRINT JOINT STATES (NEW)
        if self.latest_joint_msg is not None:
            print("Joint Positions:")
            # Zip the names and positions to print them together
            for name, pos in zip(self.latest_joint_msg.name, self.latest_joint_msg.position):
                print(f"  {name}: {pos:.4f}")
        else:
            print("Joint Positions: [Waiting for data...]")

def main():
    rclpy.init()
    node = EndEffectorPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()