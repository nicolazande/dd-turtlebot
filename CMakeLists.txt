## versione minima
cmake_minimum_required(VERSION 2.8.3)

## nome progetto (cartella dove metto tutto)
project(project)

## package richiesti
find_package(catkin REQUIRED COMPONENTS
	roscpp 
	rospy
	std_msgs
	message_generation
	sensor_msgs
	nav_msgs
	tf
	geometry_msgs
	message_filters
	rosgraph_msgs
)

## dichiaro i pacchetti necessari per i messaggi
generate_messages(
	DEPENDENCIES
	std_msgs
	rosgraph_msgs
	nav_msgs
	##geometry_msgs
)

## dipendenze dei package (importante)
catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs message_filters rosgraph_msgs nav_msgs
)

## includo le directories trovate nel find_package
include_directories(include ${catkin_INCLUDE_DIRS})

## aggiungo eseguibile robot_planner
add_executable(robot_planner src/robot_planner.cpp)
target_link_libraries(robot_planner ${catkin_LIBRARIES})
##add_dependencies(robot_planner project)

## aggiungo eseguibile left wheel
add_executable(left_wheel src/left_wheel.cpp)
target_link_libraries(left_wheel ${catkin_LIBRARIES})
##add_dependencies(left_wheel project)

## aggiungo eseguibile right wheel
add_executable(right_wheel src/right_wheel.cpp)
target_link_libraries(right_wheel ${catkin_LIBRARIES})
##add_dependencies(right_wheel project)

## aggiungo eseguibile robot_controller
add_executable(robot_controller src/robot_controller.cpp)
target_link_libraries(robot_controller ${catkin_LIBRARIES})
##add_dependencies(robot_controller project)

## aggiungo eseguibile robot_simulator
add_executable(robot_simulator src/robot_simulator.cpp)
target_link_libraries(robot_simulator ${catkin_LIBRARIES})
##add_dependencies(robot_simulator project)

# aggiungo eseguibile clock
add_executable(CLK src/CLK.cpp)
target_link_libraries(CLK ${catkin_LIBRARIES})
##add_dependencies(CLK project)
