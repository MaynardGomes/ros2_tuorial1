#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
from my_robot_interfaces.action import MoveTurtle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class TurtleController(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.cb_group_ = ReentrantCallbackGroup()
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.declare_parameter("turtle_name", rclpy.Parameter.Type.STRING)
        self.turtle_name_ = self.get_parameter("turtle_name").value
        self.get_logger().info(f"Name of turtle is {self.turtle_name_}")
        self.spawn_turtle_client_ = self.create_client(Spawn, "spawn", callback_group=self.cb_group_)
        self.kill_turtle_client_ = self.create_client(Kill, "kill", callback_group=self.cb_group_)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, self.turtle_name_ + "/cmd_vel", 10)
        
        self.spawn_turtle()
        self.move_turtle_server_ = ActionServer(self,
                                                MoveTurtle, 
                                                "move_turtle_" + self.turtle_name_,
                                                goal_callback=self.goal_callback,
                                                handle_accepted_callback=self.handle_accepted_callback,
                                                cancel_callback=self.cancel_callback,
                                                execute_callback=self.execute_callback,
                                                callback_group=ReentrantCallbackGroup()
                                                )
        
    # GOAL CAllBACK
    def goal_callback(self, goal_request: MoveTurtle.Goal):
        self.get_logger().info("Received goal")

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

            msg = Twist()
            msg.linear.x = linear_vel_x
            msg.angular.z = angular_vel_z
            self.cmd_vel_publisher_.publish(msg)
            rate.sleep()
     

    def spawn_turtle(self):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server (Spawn)")
        # self.spawn_turtle_timer_.cancel()
        request = Spawn.Request()
        request.name = self.turtle_name_
        request.x = 5.0
        request.y = 5.0
        self.get_logger().info("Trying to spwan a turtle")
        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(self.callback_spwan_turtle_server)

    def callback_spwan_turtle_server(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"{response.name} spawned")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def kill_turtle(self):
        while not self.kill_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server (Kill)")
        request = Kill.Request()
        request.name = self.turtle_name_
        self.get_logger().info("Trying to remove turtle")
        future = self.kill_turtle_client_.call_async(request)

    def callback_kill_turtle_server(self, future):
            self.get_logger().info(f"Turtle removed")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController() 
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
