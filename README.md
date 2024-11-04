## **Spawn Turtle Catch Game**

This is an engaging implementation of a spawn-and-catch game using ROS 2 and Turtlesim. The game involves a base turtle chasing and catching randomly spawned turtles. Each time a turtle is caught, a new target appears in a different random position, and the process continues.
Game Flow

    Locating Random Positions: The game determines random coordinates within the Turtlesim environment.
    Spawning New Turtles: A new turtle is spawned at a random location whenever a target is caught.
    Identifying Distance: The base turtle continuously calculates its distance from the closest target.
    Equating the Distance: The base turtle moves towards the target until it gets close enough to "catch" it.
    Killing the Turtle: Once the base turtle reaches the target, the target turtle is removed, and a new target is spawned.

Installation and Running the Game
Clone the Repository

bash

# Navigate to your ROS 2 workspace's source directory
cd ~/your_ros2_workspace/src

# Clone the repository
git clone https://github.com/yaramagdy454/turtle_game_1.git

Build the Workspace

bash

# Navigate to your ROS 2 workspace root
cd ~/your_ros2_workspace

# Build the workspace
colcon build

Source the Workspace

bash

# Source the workspace setup file
source install/setup.bash

Run the Game

bash

# Navigate to the launch directory of the package
cd ~/your_ros2_workspace/src/turtle_game_1/launch

# Launch the game
ros2 launch my_robot_bringup demo.launch.py

Code Explanation
spawner.py Code

This script manages the spawning of target turtles in random locations and publishes their information.

    Key Features:
        Uses the /spawn and /kill services to manage turtles.
        Publishes the position of each new target using a custom ROS 2 topic target_info.
        Automatically spawns a new target once the current one is destroyed.

base_controller.py Code

This node handles the movement of the base turtle and ensures it tracks and moves toward the closest target.

    Main Functions:
        update_targets(): Updates the target positions when new information is published.
        pose_callback(): Continuously receives the turtle's pose and calculates the distance to the closest target.
        find_closest_target(): Finds and returns the nearest target to the base turtle.
        remove_target(): Removes the current target once it's reached and spawns a new one.
        spawn_new_target(): Initiates the process to spawn a new target turtle.

Launch File (demo.launch.py)

This file launches the Turtlesim node along with the spawner and base_controller nodes.

    Nodes Launched:
        turtlesim_node: The main Turtlesim environment.
        spawner: The script that handles spawning and removing turtles.
        turtle_controller: The script that controls the movement of the base turtle.
