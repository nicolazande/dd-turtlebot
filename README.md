# dd-turtlebot

This repository contains the source code and files for the **dd-turtlebot** project.

![rqt_graph](https://github.com/nicolazande/dd-turtlebot/assets/115359494/0cb1f4d7-4aa8-42c0-8dfe-cf8677c0bce1)


## Setup

To set up the project, follow these steps:

1. Extract the contents of the zip file.
2. Move the 'project' folder to the standard 'catkin_ws/src' folder.
3. Ensure that the 'project' folder contains the necessary files: `CMakeLists.txt` and `package.xml`.
4. Start ROS by running the command `roscore` in a new terminal.
5. Compile the project by running the command `catkin_make` in the 'catkin_ws' directory.

## MATLAB

The path planning for this project is performed using two MATLAB scripts located in the 'project/matlab' folder:

1. **flat_planner.m**: This script performs path planning in an obstacle-free environment using the flatness form of the kinematic model.
2. **Planner.m**: This script performs path planning in a standard map that includes obstacles using the Rapidly-exploring Random Tree (RRT) algorithm.

After starting ROS (roscore), open one of the MATLAB scripts mentioned above. Modify the 'master_name' variable in the script, replacing it with the ROS master name (e.g., 'http://nicola-Precision-7730:11311'). Then, run the script. MATLAB will connect to the ROS master and start publishing the path message on the `/path` topic.

## Launch

The 'project/launch' folder contains two launch files:

1. **clock.launch**: Launches a ROS node that publishes simulated time messages on the `/clock` topic. It is important to launch the clock before running the project.
2. **project.launch**: Launches all the other nodes required for the project.

## Plot

The 'project/script' folder contains the following files:

- **plotData.py**: This Python script is used to plot the data. It compares the reference trajectory (ref) with the trajectory obtained via odometry (act).
- **flat.bag**: A recorded bag file used for plotting data in an obstacle-free environment.
- **obs_fast.bag**: A recorded bag file used for plotting data with fast obstacles.
- **obs_slow.bag**: A recorded bag file used for plotting data with slow obstacles.

To plot the data, navigate to the 'project/script' folder and run the command `./plotData.py flat.bag` in a new terminal. You can replace 'flat.bag' with 'obs_fast.bag' or 'obs_slow.bag' to plot the corresponding data.

## Map

The 'project/map' folder contains a standard map and its YAML file.

## RVIZ

The 'project/config' folder contains the parameters for RVIZ.

## Images

The 'project/img' folder contains several plots of the project's results.

## Terminal Commands

To run the project, follow these terminal commands:

1. Move the 'project' folder to the 'catkin_ws/src' directory.
2. Open a new terminal and run the command `roscore`.
3. Open another terminal and run the command `catkin_make`.
4. In the 'project/matlab' folder, open either 'Planner.m' or 'flat_planner.m', modify the 'master_name' variable, and run the script.
5. Open a new terminal and run the command `matlab -nodisplay -nosplash -nodesktop -r "run('matlab/Planner.m')"`.
6. Open a new terminal, navigate to the 'catkin_ws' directory, and run the command `source ./devel/setup.bash`. Then, launch the clock node by running `roslaunch project clock.launch`.
7. Open a new terminal, navigate to the 'catkin_ws' directory, and run the command `source ./devel/setup.bash`. Then, launch the project by running `roslaunch project project.launch`.
8. RVIZ will automatically open and display the reference trajectory versus the feedback.
9. Open a new terminal, go to the 'project/script' folder, and run the command `./plotData.py flat.bag` to plot the data. You can use the other bag files, such as 'obs_fast.bag' or 'obs_slow.bag', for plotting as well.
