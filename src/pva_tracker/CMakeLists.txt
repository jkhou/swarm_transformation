cmake_minimum_required(VERSION 2.8.3)

project(pva_tracker)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	dynamic_reconfigure
	)

generate_dynamic_reconfigure_options(
		cfg/pid.cfg
		cfg/pid_ground.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "../../devel/include"
  "include"
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(roslaunch)



add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES})
add_dependencies(tracker ${PROJECT_NAME}_gencfg)

add_executable(sim_control_test src/sim_control_test.cpp)
target_link_libraries(sim_control_test ${catkin_LIBRARIES})

add_executable(real_control_test src/real_control_test.cpp)
target_link_libraries(real_control_test ${catkin_LIBRARIES})

add_executable(straight_line src/straight_line.cpp)
target_link_libraries(straight_line ${catkin_LIBRARIES})

add_executable(min_jerk_pva src/min_jerk_pva.cpp)
target_link_libraries(min_jerk_pva ${catkin_LIBRARIES})
