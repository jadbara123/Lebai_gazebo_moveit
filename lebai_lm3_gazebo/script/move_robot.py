import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self._client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info("Action client created, waiting for server...")
        self._client.wait_for_server()
        self.get_logger().info("Server connected!")

    def send_trajectory(self, joint_names, trajectory_points):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        for positions, t in trajectory_points:
            positions = [float(p) for p in positions]
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal_msg.trajectory.points.append(point)

            self.get_logger().info(f"Sending trajectory with {len(trajectory_points)} points")
            future = self._client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        goal_result = result_future.result()

        self.get_logger().info(f"Trajectory finished with status: {goal_result.status}")



def main():
    rclpy.init()
    node = JointTrajectoryClient()

    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    # Example trajectory: 5 points over 5 seconds
    trajectory_points = [
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
        ([0.1, -0.1, 0.2, 0.0, 0.1, 0.0], 2.0),
        ([0.2, -0.2, 0.3, 0.0, 0.2, 0.0], 3.0),
        ([0.3, -0.3, 0.4, 0.0, 0.3, 0.0], 4.0),
        ([0.5, -0.5, 0.5, 0.0, 0.5, 0.0], 5.0),
    ]

    node.send_trajectory(joint_names, trajectory_points)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

