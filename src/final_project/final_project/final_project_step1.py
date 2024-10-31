#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class TurtleController(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.cb_group_ = ReentrantCallbackGroup()
        self.declare_parameter("turtle_name", rclpy.Parameter.Type.STRING)
        self.turtle_name_ = self.get_parameter("turtle_name").value
        #self.turtle_name_ = 'House'
        self.get_logger().info(f"Name of turtle is {self.turtle_name_}")
        self.spawn_turtle_client_ = self.create_client(Spawn, "spawn", callback_group=self.cb_group_)
        self.kill_turtle_client_ = self.create_client(Kill, "kill", callback_group=self.cb_group_)
        self.spawn_turtle()
        # self.spawn_turtle_timer_ = self.create_timer(10.0, self.spawn_turtle)
        # self.kill_turtle_timer_ = self.create_timer(20.0, self.kill_turtle)
        

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
            self.kill_turtle_timer_ = self.create_timer(5.0, self.kill_turtle)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def kill_turtle(self):
        while not self.kill_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server (Kill)")
        self.kill_turtle_timer_.cancel()
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
