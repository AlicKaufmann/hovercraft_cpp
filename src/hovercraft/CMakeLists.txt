include_directories(${CASADI_INCLUDE_DIR} /usr/local/include/casadi) # error, change to CASADI_INCLUDE_DIRS with an "s" at the end
include_directories(${CASADI_INCLUDE_DIR} /usr/local/include/casadi/core)

add_executable(callback src/callback.cpp)
target_link_libraries(callback ${CASADI_LIBRARIES})

add_executable(estimator src/estimator.cpp)
target_link_libraries(estimator ${CASADI_LIBRARIES})

add_executable(create_integrator_rk4 src/create_integrator_rk4.cpp)
target_link_libraries(create_integrator_rk4 ${CASADI_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${CASADI_LIBRARIES} hovercraft_library ${catkin_LIBRARIES}) 

add_executable(optimal_problem src/optimal_problem.cpp)
target_link_libraries(optimal_problem ${CASADI_LIBRARIES})

add_executable(test_file src/test_file.cpp)
target_link_libraries(test_file ${CASADI_LIBRARIES})

#if(CMAKE_COMPILER_IS_GNUXX)
#	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall"))
#endif()

