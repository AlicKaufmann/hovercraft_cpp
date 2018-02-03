include_directories(${CASADI_INCLUDE_DIR} /usr/local/include/casadi)
include_directories(${CASADI_INCLUDE_DIR} /usr/local/include/casadi/core)
message(${CASADI_INCLUDE_DIR})
#include_directories(/usr/local/include/casadi/core)

add_executable(callback src/callback.cpp)
target_link_libraries(callback ${CASADI_LIBRARIES})

add_executable(estimator src/estimator.cpp)
target_link_libraries(estimator ${CASADI_LIBRARIES})

add_executable(create_integrator_rk4 src/create_integrator_rk4.cpp)
target_link_libraries(create_integrator_rk4 ${CASADI_LIBRARIES})

add_executable(test2 src/test2.cpp)
target_link_libraries(test2 ${CASADI_LIBRARIES} hovercraft_library)

add_executable(optimal_problem src/optimal_problem.cpp)
target_link_libraries(optimal_problem ${CASADI_LIBRARIES})



