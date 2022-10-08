# ddTurtlebot
Differential drive example and results.

INSTRUCTIONS TO RUN THE PROJECT: (the full description of the project is in the Power Point)

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SETUP:
	Take the 'project' folder inside the zip file and put it inside the standard 'catkin_ws/src' folder
	There is already the CMakeList.txt and package.xml
	Start ROS (roscore) and compile the project (catkin_make)
	
MATLAB:
	The path planning is performed by two matlab scripts in folder 'project/matlab':
	
	1) flat_planner.m -> path planning in an obstacle free environment using flatness form of kinematic model
	2) Planner.m -> path planning in a standard map including obstacles using RRT (as seen during lectures)
	
	Once ROS is already running open one of the two script and change the 'master_name' variable with the ros master name (ex. 'http://nicola-Precision-7730:11311')
	Run the script -> matlab connects to ros master and starts publishing the path message on '/path' topic.
	
LAUNCH:
	In folder 'project/launch' there are two launch files:
	
	1) clock.launch -> launches a ROS node wich publishes a simulated time messages on '/clock' topic (the clock must be launched pefore the project)
	2) project.launch -> launches all the other nodes needed
	
PLOT:
	In folder 'project/script' there are both the python script 'plotData.py' and three bags recorded (flat.bag / obs_fast.bag / obs_slow.bag) used to plot the data
	In particular the reference trajectory (ref) is compared to the one obtained via odometry (act)
	
MAP:
	A standard map with it's yalm file is in the project/map folder
	
RVIZ:
	The parameters for rviz are in the folder project/config
	
IMAGES:
	In the folder project/img there are some plots of the results
	
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TERMINAL COMMANDS: (commands are between [])

	1) put 'project' folder in 'catkin_ws/src'
	2) new terminal -> [roscore]
  	3) new terminal -> [catkin_make]
  	4) in 'project/matlab' -> open 'Planner.m / flat_planner.m', change variable 'master_name' and run the script
  	5) new terminal -> [matlab -nodisplay -nosplash -nodesktop -r "run('matlab/Planner.m')"]
  	6) new terminal -> [source ./devel/setup.bash] + [roslaunch project clock.launch]
  	7) new terminal -> [source ./devel/setup.bash] + [roslaunch project project.launch]
  	8) rviz is automatically opened and shows reference vs feedback
  	9) new terminal -> go to 'project/script' folder -> [./plotData.py flat.bag] (oter bags: obs_fast.bag/obs_slow.bag) 	
