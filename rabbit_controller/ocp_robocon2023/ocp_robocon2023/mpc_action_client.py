import numpy as np
import casadi as ca


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mpc_action.action import MPCAction

class MPCActionClient(Node):
    def __init__(self):
        super().__init__('mpc_action_client')
        self.mpc_client = ActionClient(
                self,
                MPCAction,
                'mpc_action')


    def send_goal(self, goal_states):
        goal_msg = MPCAction.Goal()

        goal_msg.goal_states = goal_states

        self.mpc_client.wait_for_server()

        self._send_goal_future = self.mpc_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted:')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.goal_results))
        rclpy.shutdown()


    def feedback_callback(self, goal_feedback):
        feedback = goal_feedback.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.goal_feedback))


def main(args=None):

    goal_states = [1.0, 1.0, 0.0]

    rclpy.init(args=args)

    mpc_client = MPCActionClient()

    mpc_client.send_goal(goal_states)

    rclpy.spin(mpc_client)


if __name__ == '__main__':
    main()

