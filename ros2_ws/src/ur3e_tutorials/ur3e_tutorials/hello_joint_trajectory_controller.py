import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


class JointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def send_trajectory(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(point)

        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = JointTrajectoryClient()
    node.send_trajectory()
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
