#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus 
from my_robot_interfaces.action import MoveRobot

class MoveRobotClientNode(Node): 
    def __init__(self):
        super().__init__("move_robot_client") 
        self.move_robot_client_ = ActionClient(self, MoveRobot, "move_robot")

    def send_goal(self, position, velocity):
        #Wait for the server
        self.move_robot_client_.wait_for_server()

        # Create goal
        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity

        # Send the goal
        self.get_logger().info("Sending goal")
        self.move_robot_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)
        


    def cancel_goal(self):
        self.get_logger().info("Send cancel request")
        self.goal_handle_.cancel_goal_async()
   

    def goal_response_callback(self, future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        #self.get_logger().info(f"Status: {self.goal_handle_.accepted}")
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Result: {result.position} {result.message}")

    def goal_feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info(f"Got feedback: {current_position}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode() 
    node.send_goal(76, 7)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
