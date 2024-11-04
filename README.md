# **Spawn Turtle Catch Game**

This is an engaging implementation of a spawn-and-catch game using ROS 2 and Turtlesim. The game involves a base turtle chasing and catching randomly spawned turtles. Each time a turtle is caught, a new target appears in a different random position, and the process continues.


## **Installation and Running the Game**
### Clone the Repository

cd ~/your_ros2_workspace/src

git clone https://github.com/yaramagdy454/turtle_game_1.git

cd ~/your_ros2_workspace

colcon build

### **Source the Workspace**
source install/setup.bash

### **Run the Game**

cd ~/your_ros2_workspace/src/turtle_game_1/launch
ros2 launch my_robot_bringup demo.launch.py

## **Code Explanation**
### spawner.py Code

This script manages the spawning of target turtles in random locations and publishes their information.

    Key Features:
        Uses the /spawn and /kill services to manage turtles.
        Publishes the position of each new target using a custom ROS 2 topic target_info.
        Automatically spawns a new target once the current one is destroyed.

### base_controller.py Code

This node handles the movement of the base turtle and ensures it tracks and moves toward the closest target.

    Main Functions:
        update_targets(): Updates the target positions when new information is published.
        pose_callback(): Continuously receives the turtle's pose and calculates the distance to the closest target.
        find_closest_target(): Finds and returns the nearest target to the base turtle.
        remove_target(): Removes the current target once it's reached and spawns a new one.
        spawn_new_target(): Initiates the process to spawn a new target turtle.

### Launch File (demo.launch.py)

This file launches the Turtlesim node along with the spawner and base_controller nodes.

    Nodes Launched:
        turtlesim_node: The main Turtlesim environment.
        spawner: The script that handles spawning and removing turtles.
        turtle_controller: The script that controls the movement of the base turtle.
