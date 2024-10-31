#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from my_robot_interfaces.action import MoveTurtle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class TurtleBotController(LifecycleNode): 
    def __init__(self):
        super().__init__("turtle_bot_controller") 
        self.cb_group_ = ReentrantCallbackGroup()
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.server_activated = False
        
        # self.get_logger().info(f"Name of turtle is {self.turtle_name_}")
        
        
     # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN ON_CONFIGURE")
        self.cmd_vel_publisher_ = self.create_lifecycle_publisher(Twist, "cmd_vel", 10)
        self.move_turtle_server_ = ActionServer(self,
                                                MoveTurtle, 
                                                "move_turtle",
                                                goal_callback=self.goal_callback,
                                                handle_accepted_callback=self.handle_accepted_callback,
                                                cancel_callback=self.cancel_callback,
                                                execute_callback=self.execute_callback,
                                                callback_group=ReentrantCallbackGroup())
        
        return TransitionCallbackReturn.SUCCESS
    
    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN ON_ACTIVATE")
        self.server_activated = True
        return super().on_activate(previous_state)
    
     # Deactivate/Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN ON_DEACTIVATE")
        self.server_activated = False
        return super().on_deactivate(previous_state)

    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN ON_CLEANUP")
        self.move_turtle_server_.destroy()
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        
        return TransitionCallbackReturn.SUCCESS
    
    # Cleanup everything
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN ON_SHUTDOWN")
        self.server_activated = False
        self.move_turtle_server_.destroy()
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        return TransitionCallbackReturn.SUCCESS
    
    # GOAL CAllBACK
    def goal_callback(self, goal_request: MoveTurtle.Goal):
        self.get_logger().info("Received goal")

        # Check if server is active
        if not self.server_activated:
            self.get_logger().info("Server is inactive")
            return GoalResponse.REJECT

        # Policy: refuse new goal if current goal is active
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("A goal is already active, rejecting new goal")
                return GoalResponse.REJECT

        # Validate goal request
        #if goal_request.linear_vel_x in range(0.0, 3.0) and goal_request.angular_vel_z in range(0.0, 3.0) and goal_request.duration_sec in range(1, 10):
        if goal_request.linear_vel_x < 0.0 and goal_request.angular_vel_z < 0.0 and goal_request.duration_sec < 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT 
    
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
        
        
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Handling the goal")
        with self.goal_lock_:
            goal_handle.execute()

    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        result = MoveTurtle.Result()

        # get request from goal
        linear_vel_x =  goal_handle.request.linear_vel_x
        angular_vel_z =  goal_handle.request.angular_vel_z
        duration_sec =  goal_handle.request.duration_sec

        # Create a rate timer
        rate = self.create_rate(10)

        # Execute the action
        self.get_logger().info("Executing the goal")
        now = int(time.time())


        while rclpy.ok():
            if (int(time.time()) - now >= duration_sec):
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_publisher_.publish(msg)
                result.success = True
                result.message = "Success"
                goal_handle.succeed()
                return result
            
            if not self.server_activated:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_publisher_.publish(msg)
                result.success = False
                result.message = "Aborted because server was deactivated"
                goal_handle.abort()
                return result

            msg = Twist()
            msg.linear.x = linear_vel_x
            msg.angular.z = angular_vel_z
            self.cmd_vel_publisher_.publish(msg)
            rate.sleep()
     

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController() 
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
