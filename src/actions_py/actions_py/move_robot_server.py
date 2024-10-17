#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class MoveRobotServerNode(Node): 
    def __init__(self):
        super().__init__("move_robot_server") 
        self.robot_position_ = 50
        self.move_robot_server_ = ActionServer(self, 
                                                MoveRobot, 
                                                "move_robot", 
                                                execute_callback=self.execute_callback
                                                )

        self.get_logger().info("Move robot server has been started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # get request from goal
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        # execute the action
        self.get_logger().info("Executing the goal")
        
        while rclpy.ok():
            diff = goal_position - self.robot_position_

            if abs(diff) < velocity:
                velocity = abs(diff)

            if diff == 0:
                # Target reached
                self.get_logger().info("Target Reached")
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
    
            elif diff > 0:
                # Add velocity
                self.robot_position_ += velocity

            else:
                # Subtract velocity
                self.robot_position_ -= velocity

            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"Current position: {self.robot_position_}")
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
