# toplevel CMakeLists.txt
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# put debugging symbols
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/alic/casadi/cmake)

find_package(CASADI REQUIRED)
find_package(IPOPT REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	)
 

#find_package(roscpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(message_generation REQUIRED)

#Libraries for linking the ros and boost commands.


add_subdirectory(hovercraft)
add_subdirectory(mylibrary)

include_directories(${catkin_INLUDE_DIRS})
