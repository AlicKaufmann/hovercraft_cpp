# CMakeLists.txt for the library

# add the hovercraft library
add_library(hovercraft_library STATIC
	src/Kalman_filter.cpp
	src/Hovercraft_model.cpp
	src/Reference.cpp
	src/Cost.cpp
	src/create_integrator.cpp
	src/MPC.cpp
	src/HC_node.cpp
	)


# include directories for the library
target_include_directories(hovercraft_library PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
	$<BUILD_INTERFACE:/opt/ros/lunar/include> # why does this work and $<BUILD_INTERFACE:${catkin_INCLUDE_DIRS}> not?
	#$<BUILD_INTERFACE:${ROSCPP_INCLUDE_DIRS}> 
	#$<BUILD_INTERFACE:${catkin_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>
	)

