# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/cg/cgue2/02_whitted

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cg/cgue2/02_whitted/build

# Include any dependencies generated for this target.
include /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/depend.make

# Include the progress variables for this target.
include /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/progress.make

# Include the compile flags for this target's objects.
include /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/flags.make

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/flags.make
/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o: /home/cg/cgue2/cglib/lib/glew/src/glew.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o"
	cd /home/cg/cgue2/cglib/build/glew && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glew32mxs.dir/src/glew.c.o   -c /home/cg/cgue2/cglib/lib/glew/src/glew.c

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew32mxs.dir/src/glew.c.i"
	cd /home/cg/cgue2/cglib/build/glew && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glew/src/glew.c > CMakeFiles/glew32mxs.dir/src/glew.c.i

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew32mxs.dir/src/glew.c.s"
	cd /home/cg/cgue2/cglib/build/glew && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glew/src/glew.c -o CMakeFiles/glew32mxs.dir/src/glew.c.s

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.requires

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.provides: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/build.make /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.provides

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.provides.build: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o


# Object files for target glew32mxs
glew32mxs_OBJECTS = \
"CMakeFiles/glew32mxs.dir/src/glew.c.o"

# External object files for target glew32mxs
glew32mxs_EXTERNAL_OBJECTS =

/home/cg/cgue2/cglib/build/glew/libglew32mxs.a: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o
/home/cg/cgue2/cglib/build/glew/libglew32mxs.a: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/build.make
/home/cg/cgue2/cglib/build/glew/libglew32mxs.a: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libglew32mxs.a"
	cd /home/cg/cgue2/cglib/build/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew32mxs.dir/cmake_clean_target.cmake
	cd /home/cg/cgue2/cglib/build/glew && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glew32mxs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/build: /home/cg/cgue2/cglib/build/glew/libglew32mxs.a

.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/build

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/requires: /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/src/glew.c.o.requires

.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/requires

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/clean:
	cd /home/cg/cgue2/cglib/build/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew32mxs.dir/cmake_clean.cmake
.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/clean

/home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/depend:
	cd /home/cg/cgue2/02_whitted/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cg/cgue2/02_whitted /home/cg/cgue2/cglib/lib/glew /home/cg/cgue2/02_whitted/build /home/cg/cgue2/cglib/build/glew /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : /home/cg/cgue2/cglib/build/glew/CMakeFiles/glew32mxs.dir/depend

