# **Spawm Turtle Catch Game **


This is a fun implementation of spawm game done using ROS2 Turtlesim.

Initially a turtle will be there which is the base turtle of the game. A new turtle will be spawned to which the base turtle will go to the goal position to catch it. Everytime we catch a turtle a new turtle is spawned in a random position. This process repeats continuously.
<p>

  
<h3><b>Flow of program</b></h3>
  <br><ul><li>Locating random positions</li><br>
  <li>Spawning new turtle</li><br>
  <li>Identifying distance</li><br>
  <li>Equating the distance </li><br>
  <li>Killing the turtle</li> <br></ul></p>
  
<ol>
  <p>










/**run game**

//clone the repo:
cd ~/your_ros2_workspace/src
git clone https://github.com/yaramagdy454/turtle_game_1.git

//Build the Workspace:
cd ~/your_ros2_workspace
colcon build

//source the workspace:
source install/setup.bash

//run:
cd ~/your_ros2_workspace/src/turtle_game_1/launch
ros2 launch my_robot_bringup demo.launch.py
