# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alic/casadi_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alic/casadi_ws/build

# Include any dependencies generated for this target.
include hovercraft/CMakeFiles/optimal_problem.dir/depend.make

# Include the progress variables for this target.
include hovercraft/CMakeFiles/optimal_problem.dir/progress.make

# Include the compile flags for this target's objects.
include hovercraft/CMakeFiles/optimal_problem.dir/flags.make

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o: hovercraft/CMakeFiles/optimal_problem.dir/flags.make
hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o: /home/alic/casadi_ws/src/hovercraft/src/optimal_problem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alic/casadi_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o"
	cd /home/alic/casadi_ws/build/hovercraft && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o -c /home/alic/casadi_ws/src/hovercraft/src/optimal_problem.cpp

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.i"
	cd /home/alic/casadi_ws/build/hovercraft && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alic/casadi_ws/src/hovercraft/src/optimal_problem.cpp > CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.i

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.s"
	cd /home/alic/casadi_ws/build/hovercraft && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alic/casadi_ws/src/hovercraft/src/optimal_problem.cpp -o CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.s

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.requires:

.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.requires

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.provides: hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.requires
	$(MAKE) -f hovercraft/CMakeFiles/optimal_problem.dir/build.make hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.provides.build
.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.provides

hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.provides.build: hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o


# Object files for target optimal_problem
optimal_problem_OBJECTS = \
"CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o"

# External object files for target optimal_problem
optimal_problem_EXTERNAL_OBJECTS =

hovercraft/optimal_problem: hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o
hovercraft/optimal_problem: hovercraft/CMakeFiles/optimal_problem.dir/build.make
hovercraft/optimal_problem: /usr/local/lib/libcasadi.so.3.4
hovercraft/optimal_problem: hovercraft/CMakeFiles/optimal_problem.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alic/casadi_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable optimal_problem"
	cd /home/alic/casadi_ws/build/hovercraft && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optimal_problem.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hovercraft/CMakeFiles/optimal_problem.dir/build: hovercraft/optimal_problem

.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/build

hovercraft/CMakeFiles/optimal_problem.dir/requires: hovercraft/CMakeFiles/optimal_problem.dir/src/optimal_problem.cpp.o.requires

.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/requires

hovercraft/CMakeFiles/optimal_problem.dir/clean:
	cd /home/alic/casadi_ws/build/hovercraft && $(CMAKE_COMMAND) -P CMakeFiles/optimal_problem.dir/cmake_clean.cmake
.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/clean

hovercraft/CMakeFiles/optimal_problem.dir/depend:
	cd /home/alic/casadi_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alic/casadi_ws/src /home/alic/casadi_ws/src/hovercraft /home/alic/casadi_ws/build /home/alic/casadi_ws/build/hovercraft /home/alic/casadi_ws/build/hovercraft/CMakeFiles/optimal_problem.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hovercraft/CMakeFiles/optimal_problem.dir/depend

