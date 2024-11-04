#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import String
from functools import partial
import random
import math
import json

class SpawnerNode(Node):
    def __init__(self):
        super().__init__("spawner")
        self.targets = {}
        self.target_publisher = self.create_publisher(String, "target_info", 10)
        self.spawn_new_target()
        self.get_logger().info("Spawner has been started.")

    def spawn_new_target(self):
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0, 2 * math.pi)
        request.name = "target_" + str(random.randint(0, 1000))

        future = client.call_async(request)
        future.add_done_callback(partial(self.handle_spawn_response, request))

    def handle_spawn_response(self, request, future):
        try:
            response = future.result()
            target_info = {
                'x': request.x,
                'y': request.y,
                'name': response.name
            }
            self.targets[response.name] = target_info
            self.get_logger().info(f"Spawned new target: {response.name} at ({request.x}, {request.y})")

            # Publish target information
            self.target_publisher.publish(String(data=json.dumps(target_info)))
        except Exception as e:
            self.get_logger().error(f"Failed to spawn target: {e}")

    def remove_target(self, target_name):
        client = self.create_client(Kill, "/kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service...")

        request = Kill.Request()
        request.name = target_name
        future = client.call_async(request)
        future.add_done_callback(partial(self.handle_kill_response, target_name))

    def handle_kill_response(self, target_name, future):
        try:
            _ = future.result()
            self.get_logger().info(f"Target '{target_name}' destroyed.")
            del self.targets[target_name]
            self.spawn_new_target()  # Spawn a new target after killing the old one
        except Exception as e:
            self.get_logger().error(f"Failed to destroy target '{target_name}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()



