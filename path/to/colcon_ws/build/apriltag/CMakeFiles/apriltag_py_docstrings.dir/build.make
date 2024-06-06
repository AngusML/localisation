# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/angus/path/to/colcon_ws/src/apriltag

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/angus/path/to/colcon_ws/build/apriltag

# Utility rule file for apriltag_py_docstrings.

# Include any custom commands dependencies for this target.
include CMakeFiles/apriltag_py_docstrings.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/apriltag_py_docstrings.dir/progress.make

CMakeFiles/apriltag_py_docstrings: apriltag_detect_docstring.h
CMakeFiles/apriltag_py_docstrings: apriltag_py_type_docstring.h

apriltag_detect_docstring.h: /home/angus/path/to/colcon_ws/src/apriltag/CMake/vtkEncodeString.cmake
apriltag_detect_docstring.h: /home/angus/path/to/colcon_ws/src/apriltag/apriltag_detect.docstring
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/angus/path/to/colcon_ws/build/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating apriltag_detect_docstring.h, apriltag_detect_docstring.cxx"
	/usr/bin/cmake -Dsource_dir=/home/angus/path/to/colcon_ws/src/apriltag -Dbinary_dir=/home/angus/path/to/colcon_ws/build/apriltag -Dsource_file=/home/angus/path/to/colcon_ws/src/apriltag/apriltag_detect.docstring -Doutput_name=apriltag_detect_docstring -Dexport_symbol= -Dexport_header= -Dabi_mangle_symbol_begin= -Dabi_mangle_symbol_end= -Dabi_mangle_header= -Dbinary=FALSE -Dnul_terminate=FALSE -D_vtk_encode_string_run=ON -P /home/angus/path/to/colcon_ws/src/apriltag/CMake/vtkEncodeString.cmake

apriltag_detect_docstring.cxx: apriltag_detect_docstring.h
	@$(CMAKE_COMMAND) -E touch_nocreate apriltag_detect_docstring.cxx

apriltag_py_type_docstring.h: /home/angus/path/to/colcon_ws/src/apriltag/CMake/vtkEncodeString.cmake
apriltag_py_type_docstring.h: /home/angus/path/to/colcon_ws/src/apriltag/apriltag_py_type.docstring
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/angus/path/to/colcon_ws/build/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating apriltag_py_type_docstring.h, apriltag_py_type_docstring.cxx"
	/usr/bin/cmake -Dsource_dir=/home/angus/path/to/colcon_ws/src/apriltag -Dbinary_dir=/home/angus/path/to/colcon_ws/build/apriltag -Dsource_file=/home/angus/path/to/colcon_ws/src/apriltag/apriltag_py_type.docstring -Doutput_name=apriltag_py_type_docstring -Dexport_symbol= -Dexport_header= -Dabi_mangle_symbol_begin= -Dabi_mangle_symbol_end= -Dabi_mangle_header= -Dbinary=FALSE -Dnul_terminate=FALSE -D_vtk_encode_string_run=ON -P /home/angus/path/to/colcon_ws/src/apriltag/CMake/vtkEncodeString.cmake

apriltag_py_type_docstring.cxx: apriltag_py_type_docstring.h
	@$(CMAKE_COMMAND) -E touch_nocreate apriltag_py_type_docstring.cxx

apriltag_py_docstrings: CMakeFiles/apriltag_py_docstrings
apriltag_py_docstrings: apriltag_detect_docstring.cxx
apriltag_py_docstrings: apriltag_detect_docstring.h
apriltag_py_docstrings: apriltag_py_type_docstring.cxx
apriltag_py_docstrings: apriltag_py_type_docstring.h
apriltag_py_docstrings: CMakeFiles/apriltag_py_docstrings.dir/build.make
.PHONY : apriltag_py_docstrings

# Rule to build all files generated by this target.
CMakeFiles/apriltag_py_docstrings.dir/build: apriltag_py_docstrings
.PHONY : CMakeFiles/apriltag_py_docstrings.dir/build

CMakeFiles/apriltag_py_docstrings.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/apriltag_py_docstrings.dir/cmake_clean.cmake
.PHONY : CMakeFiles/apriltag_py_docstrings.dir/clean

CMakeFiles/apriltag_py_docstrings.dir/depend:
	cd /home/angus/path/to/colcon_ws/build/apriltag && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/angus/path/to/colcon_ws/src/apriltag /home/angus/path/to/colcon_ws/src/apriltag /home/angus/path/to/colcon_ws/build/apriltag /home/angus/path/to/colcon_ws/build/apriltag /home/angus/path/to/colcon_ws/build/apriltag/CMakeFiles/apriltag_py_docstrings.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/apriltag_py_docstrings.dir/depend

