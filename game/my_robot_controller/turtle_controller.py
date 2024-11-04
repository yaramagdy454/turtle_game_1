#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.srv import Kill, Spawn
import math
import json
from functools import partial
import random

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__("base_controller")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.target_subscriber_ = self.create_subscription(String, "target_info", self.update_targets, 10)
        self.targets = {}
        self.get_logger().info("Base controller has been started.")

    def update_targets(self, msg: String):
        target_info = json.loads(msg.data)
        self.targets[target_info['name']] = target_info
        self.get_logger().info(f"Updated targets: {self.targets}")

    def pose_callback(self, pose: Pose):
        if not self.targets:
            self.get_logger().warn("No targets available!")
            return

        closest_target = self.find_closest_target(pose)
        dx = closest_target['x'] - pose.x
        dy = closest_target['y'] - pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        self.get_logger().info(f"Distance to target: {distance}")

        if distance < 0.5:
            self.get_logger().info(f"Target reached at ({closest_target['x']}, {closest_target['y']})!")
            self.remove_target(closest_target['name'])
            return

        angle_to_target = math.atan2(dy, dx)
        cmd = Twist()
        cmd.linear.x = 1.0  
        cmd.angular.z = 2.0 * (angle_to_target - pose.theta)

        self.get_logger().info(f"Publishing cmd_vel: linear.x = {cmd.linear.x}, angular.z = {cmd.angular.z}")
        self.cmd_vel_publisher_.publish(cmd)

    def find_closest_target(self, pose):
        return min(self.targets.values(), key=lambda target: math.sqrt((target['x'] - pose.x) ** 2 + (target['y'] - pose.y) ** 2))

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
            future.result()
            self.get_logger().info(f"Successfully destroyed target: {target_name}")
            del self.targets[target_name] 
            # Now spawn a new target
            self.spawn_new_target()
        except Exception as e:
            self.get_logger().error(f"Failed to destroy target '{target_name}': {e}")

    def spawn_new_target(self):
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)  # Adjust spawn coordinates as needed
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
            # Optionally, publish the new target information
            self.target_subscriber_.publish(String(data=json.dumps(target_info)))
        except Exception as e:
            self.get_logger().error(f"Failed to spawn new target: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()






