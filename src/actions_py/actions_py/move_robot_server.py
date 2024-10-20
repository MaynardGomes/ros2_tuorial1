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
        self.goal_lock_ = threading.Lock()
        self.robot_position_ = 50
        self.goal_handle_ : ServerGoalHandle = None

        self.move_robot_server_ = ActionServer(self, 
                                                MoveRobot, 
                                                "move_robot", 
                                                goal_callback=self.goal_callback,
                                                cancel_callback=self.cancel_callback,
                                                execute_callback=self.execute_callback,
                                                callback_group=ReentrantCallbackGroup())

        self.get_logger().info("Move robot server has been started")


    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received goal")

        if goal_request.position < 0 and goal_request.position > 100 and goal_request.velocity <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        with self.goal_lock_:
            #self.get_logger().error(f"Goal Handle status {self.goal_handle_}")
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
            

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # get request from goal
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        # execute the action
        self.get_logger().info("Executing the goal")
        
        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                if goal_position == self.robot_position_:
                    self.get_logger().info("Goal reached")
                    goal_handle.succeed()
                    result.position = self.robot_position_
                    result.message = "Success"
                else:
                    self.get_logger().info("Canceling the goal")
                    goal_handle.canceled()
                    result.position = self.robot_position_
                    result.message = "Canceled"
                    
                return result
            
            if not goal_handle.is_active:
                # Prempting Goal
                self.get_logger().info("Preemting goal")
                result.position = self.robot_position_
                result.message = "Premted by another goal"
                return result
            
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
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
