# toplevel CMakeLists.txt
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# put debugging symbols
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/alic/casadi/cmake)

find_package(CASADI REQUIRED)
find_package(IPOPT REQUIRED)

add_subdirectory(hovercraft)
add_subdirectory(mylibrary)

message("CXX machin truc" ${CMAKE_CXX_COMPILER})
message(" lautre truc" ${CMAKE_CXX_COMPILER_ID})
