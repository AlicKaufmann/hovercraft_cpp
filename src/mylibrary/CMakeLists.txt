# CMakeLists.txt for the library

# add the hovercraft library
add_library(hovercraft_library STATIC
	src/Kalman_filter.cpp
	src/Hovercraft_model.cpp
	src/Reference.cpp
	src/Cost.cpp
	src/create_integrator_cost.cpp
	)

# include directories for the library
target_include_directories(hovercraft_library PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
	$<INSTALL_INTERFACE:include>)

