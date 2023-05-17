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


    def send_goal(self, goal):
        self.get_logger().info('Waiting for mpc server')
        self.mpc_client.wait_for_server()

        goal_msg = MPCAction.Goal()
        goal_msg.goal_states = goal


        self.send_goal_future = self.mpc_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)


        self.send_goal_future.add_done_callback(self.goal_response_callback)


   
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.goal_handle = goal_handle

        self.get_logger().info('Goal accepted')

        #self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Canceling goal')

        # Cancel the goal
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        self.timer.cancel()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.goal_results))
        rclpy.shutdown()


    def feedback_callback(self, goal_feedback):
        feedback = goal_feedback.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.goal_feedback))


def main(args=None):

    goal_states = [2.0, 0.0, 0.0]

    rclpy.init(args=args)

    mpc_client = MPCActionClient()

    mpc_client.send_goal(goal_states)

    rclpy.spin(mpc_client)


if __name__ == '__main__':
    main()

