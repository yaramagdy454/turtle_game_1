from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  
    ld = LaunchDescription()

    turtle_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )

    talker_node = Node(
        package="my_robot_controller",
        executable="spawner",
    )

    listener_node = Node(
        package="my_robot_controller",
        executable="turtle_controller",
        output="screen"
    )

    ld.add_action(turtle_node)
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    
    return ld
