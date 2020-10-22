# Robot Delivery System

## Overview
This project implements a multi-agent robot delivery system using ROS, Gazebo, PDDL, and the Metric-FF planner. In this environment, we are given a set of houses, a set of robots, and a set of packages. Each house has a delivery location and each package has a load location. Packages also have a size and a target house for delivery. The task for the robots is to coordinate loading the packages and delivering them to their respective house delivery locations. Each robot may carry multiple packages, but the total packages carried by each robot is restricted by the sum of the sizes of all packages the robot is carrying. Each robot is further restricted in that it cannot move between locations once a limit of movement cost has been reached.

## Setup and Execution
These steps assume running on a freshly installed Ubuntu 16.04.6.
### Setup ROS and Gazebo
From the command line, execute the following:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Setup Workspace
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH /home/YOURUSER/catkin_ws/src:/opt/ros/kinetic/share
```
Note: You need to replace YOURUSER in the command above with the name for your user folder.

### Setup Turtlebot
```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
catkin_make
```

### Setup Project
Copy the project folder, group_16, into your ~/catkin_ws/src/ folder, then run the following commands:  
```
cd ~/catkin_ws/src/group_16
./env_setup.sh
cd ~/catkin_ws
catkin_make
chmod u+x ~/catkin_ws/src/group_16/*.sh
chmod u+x ~/catkin_ws/src/group_16/scipts/*.py
```

### Running the Project
Everytime you open a new terminal, you must run the following command:
```
source ~/catkin_ws/devel/setup.bash
```
We have included 2 scripts for running the code. From the project's root folder you can run either of the following scripts:
```
./launch.sh
./launch_headless.sh
```
Both of these scripts will launch roscore and all services, so you need to make sure they are not already running.  
Note: Depending on execution times, you may need to adjust the sleep durations in the scripts if processes are taking too long to start up.


#### Command Descriptions
```
rosrun group_16 server.py -robots 1 -houses 1 -streets 1 -packages 1 -headless true
```
Launches the server for the environment.  
-robots - The number of robots to create in the environment  
-houses - The number of houses per street  
-streets - The number of streets  
-packages - The number of packages to be delivered. Note: This must not be higher than (houses * streets)  
-headless true - If provided then server will run separate from Gazebo


```
roslaunch group_16 neighborhood.launch
```
Launches the Gazebo environment.


```
rosrun group_16 move_tbot3.py -id 0
```
Launches the controller for a robot.  
-id - The ID for which robot to control. The IDs start at 0 and range to (n-1) where n is the number of robots.  
Note: You must launch this for each robot in the environment.


```
cd ~/catkin_ws/src/group_16
./generate_plan.sh
```
This generates a plan. If no plan can be found, the resulting file will be empty.  
Note: This command must be run from the root folder of the project.


```
rosrun group_16 execute_plan.py
```
This parses the generated plan and executes it.  


```
rosrun group_16 test.py -command is_terminal_state
```
This command prints out either True or False depending on whether the actions taken from execute_plan.py reached the goal state.


## Changes
- action_config.json  
We changed the names of the pick and place actions to load and deliver. We also made actions deterministic.

- domain.pddl  
This file provides the domain description for the Metric-FF planner. This won't work with FD or FF due to the use of fluents.  

- generate_plan.sh
This shell script will runn Metric-FF and generate a plan in problem.pddl.soln. This file will be empty if no valid plan could be found.

- launch.sh  
This script launches all of the scripts necessary to generate a plan and execute it in Gazebo. Variables are provided at the top of the file for customizing the number of houses, streets, robots, and packages.

- launch_headless.sh  
Same as for launch.sh except this does not simulate execution in Gazebo.

- problem.pddl  
This file is dynamically generated by problem_generator.py when server.py is run.

- readme.md.old  
The original readme.md file

### /launch
- neighborhood.launch  
This is a file generated when running server.py to allow for a variable number of robots in the environment.

- one_robot.launch  
This file includes the details for a single robot and is included for each robot in neighborhood.launch

### /planners
- Metric-FF  
This folder includes the Metric-FF planner that we use to solve our planning problem. We chose Metric-FF because it provides access to fluents. However, Metric-FF does not provide an output file for the plan, so we use the grep tool to create a file that only includes the steps of the plan. See generate_plan.sh for command line details.

### /worlds
- empty_world.sdf  
We increased the size of the floor to avoid the robot falling off the edge.

### /scripts
- action_server.py  
This file has been heavily modified from the original. We made significant modifications to the state representation which led to significant changes to all of the functions for the RobotActionServer class. A new method was added for get_path that's a modified version of the A* search from the homework 1 search problems.

- environment_api.py
We added a new function, get_path, which given a robot's ID and a target location as a 2-tuple provides a sequence of actions for that robot to reach the target location.

- execute_plan.py  
This is a new script that parses the plan generated by Metric-FF and executes it through environment_api.py.

- move_tbot3.py  
We updated this script to include the robot's ID in the ROS services it subscribes to and publishes. It now accepts a command line argument, -id, that is given as an integer and used to determine which robot it will control.

- neighborhood.py  
This file based on mazeGenerator.py and is used to create the world file for our environment.

- pid.py  
We made adjustments to include the robot's ID in the name for the services used.

- problem_generator.py  
We made changes so that the file generated a problem.pddl file that was consistent with the new domain and worked with Metric-FF.

- server.py  
We changed the command line arguments and the initialization procedure for starting the server. During setup the launch and world files are generated and the RobotActionServer is initialized.

- test.py  
New file created for testing API functions.

## Additional References

https://github.com/AAIR-lab/group_16 - This is the original project that our project was based on.
http://wiki.ros.org/kinetic/Installation/Ubuntu - Instructions for installing ROS on Ubuntu.
http://wiki.ros.org/catkin/Tutorials/create_a_workspace - Instructions for setting up a workspace.
http://users.cecs.anu.edu.au/~patrik/pddlman/writing.html - Provides details on PDDL including how to implement fluents.  
https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html - Contains the source code for Metric-FF and a brief overview.
https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/ - To put multiple robots in gazebo UI.
