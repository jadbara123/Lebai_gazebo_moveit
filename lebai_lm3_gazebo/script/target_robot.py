import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from lebai_lm3_gazebo.srv import ExecuteWaypoints 

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.cli = self.create_client(ExecuteWaypoints, '/execute_waypoints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = ExecuteWaypoints.Request()

    def send_request(self):
        # Define Waypoint 1
        p1 = Pose()
        p1.position.x = 0.3
        p1.position.y = -0.3
        p1.position.z = 0.3
        p1.orientation.y = 1.0 # Point down

        # Define Waypoint 2 (e.g., Move UP)
        p2 = Pose()
        p2.position.x = 0.3
        p2.position.y = -0.3
        p2.position.z = 0.45
        p2.orientation.y = 1.0 

        # Add to list
        self.req.poses = [p1, p2]

        # Call Service
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = WaypointClient()
    
    response = client.send_request()
    
    if response.success:
        print(f"SUCCESS: {response.message}")
    else:
        print(f"FAILED: {response.message}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()