import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint


class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_client')

        # Create an action client for the MoveGroup action server
        self._client = ActionClient(self, MoveGroup, '/move_group')

        # Send the goal after a short delay to allow system startup
        self.timer = self.create_timer(1.0, self.send_goal_once)
        self.sent = False

    def send_goal_once(self):
        if self.sent:
            return

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return

        # Create a motion planning request for a simple joint goal
        request = MotionPlanRequest()
        request.group_name = 'manipulator'
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Define joint constraints (adjust joint names and values for your robot)
        joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        joint_values = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

        constraint = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)
        request.goal_constraints.append(constraint)

        # Wrap the planning request into a MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.get_logger().info("Sending goal to MoveGroup...")
        self._client.send_goal_async(goal_msg)
        self.sent = True


def main():
    rclpy.init()
    node = MoveGroupClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()