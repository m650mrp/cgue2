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
include /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/depend.make

# Include the progress variables for this target.
include /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/progress.make

# Include the compile flags for this target's objects.
include /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o: /home/cg/cgue2/cglib/lib/glfw/src/context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/context.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/context.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/context.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/context.c > CMakeFiles/glfw.dir/context.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/context.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/context.c -o CMakeFiles/glfw.dir/context.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o: /home/cg/cgue2/cglib/lib/glfw/src/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/init.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/init.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/init.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/init.c > CMakeFiles/glfw.dir/init.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/init.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/init.c -o CMakeFiles/glfw.dir/init.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o: /home/cg/cgue2/cglib/lib/glfw/src/input.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/input.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/input.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/input.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/input.c > CMakeFiles/glfw.dir/input.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/input.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/input.c -o CMakeFiles/glfw.dir/input.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: /home/cg/cgue2/cglib/lib/glfw/src/monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/monitor.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/monitor.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/monitor.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/monitor.c > CMakeFiles/glfw.dir/monitor.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/monitor.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/monitor.c -o CMakeFiles/glfw.dir/monitor.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o: /home/cg/cgue2/cglib/lib/glfw/src/window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/window.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/window.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/window.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/window.c > CMakeFiles/glfw.dir/window.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/window.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/window.c -o CMakeFiles/glfw.dir/window.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o: /home/cg/cgue2/cglib/lib/glfw/src/x11_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_init.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/x11_init.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_init.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/x11_init.c > CMakeFiles/glfw.dir/x11_init.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_init.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/x11_init.c -o CMakeFiles/glfw.dir/x11_init.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o: /home/cg/cgue2/cglib/lib/glfw/src/x11_monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_monitor.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/x11_monitor.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_monitor.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/x11_monitor.c > CMakeFiles/glfw.dir/x11_monitor.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_monitor.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/x11_monitor.c -o CMakeFiles/glfw.dir/x11_monitor.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o: /home/cg/cgue2/cglib/lib/glfw/src/x11_window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/x11_window.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/x11_window.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/x11_window.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/x11_window.c > CMakeFiles/glfw.dir/x11_window.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/x11_window.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/x11_window.c -o CMakeFiles/glfw.dir/x11_window.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o: /home/cg/cgue2/cglib/lib/glfw/src/xkb_unicode.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/xkb_unicode.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/xkb_unicode.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/xkb_unicode.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/xkb_unicode.c > CMakeFiles/glfw.dir/xkb_unicode.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/xkb_unicode.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/xkb_unicode.c -o CMakeFiles/glfw.dir/xkb_unicode.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o: /home/cg/cgue2/cglib/lib/glfw/src/linux_joystick.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/linux_joystick.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/linux_joystick.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/linux_joystick.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/linux_joystick.c > CMakeFiles/glfw.dir/linux_joystick.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/linux_joystick.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/linux_joystick.c -o CMakeFiles/glfw.dir/linux_joystick.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o: /home/cg/cgue2/cglib/lib/glfw/src/posix_time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/posix_time.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/posix_time.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/posix_time.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/posix_time.c > CMakeFiles/glfw.dir/posix_time.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/posix_time.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/posix_time.c -o CMakeFiles/glfw.dir/posix_time.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: /home/cg/cgue2/cglib/lib/glfw/src/posix_tls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/posix_tls.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/posix_tls.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/posix_tls.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/posix_tls.c > CMakeFiles/glfw.dir/posix_tls.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/posix_tls.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/posix_tls.c -o CMakeFiles/glfw.dir/posix_tls.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o


/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/flags.make
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o: /home/cg/cgue2/cglib/lib/glfw/src/glx_context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/glx_context.c.o   -c /home/cg/cgue2/cglib/lib/glfw/src/glx_context.c

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/glx_context.c.i"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cg/cgue2/cglib/lib/glfw/src/glx_context.c > CMakeFiles/glfw.dir/glx_context.c.i

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/glx_context.c.s"
	cd /home/cg/cgue2/cglib/build/glfw/src && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cg/cgue2/cglib/lib/glfw/src/glx_context.c -o CMakeFiles/glfw.dir/glx_context.c.s

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires:

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires
	$(MAKE) -f /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides.build
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.provides.build: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o


# Object files for target glfw
glfw_OBJECTS = \
"CMakeFiles/glfw.dir/context.c.o" \
"CMakeFiles/glfw.dir/init.c.o" \
"CMakeFiles/glfw.dir/input.c.o" \
"CMakeFiles/glfw.dir/monitor.c.o" \
"CMakeFiles/glfw.dir/window.c.o" \
"CMakeFiles/glfw.dir/x11_init.c.o" \
"CMakeFiles/glfw.dir/x11_monitor.c.o" \
"CMakeFiles/glfw.dir/x11_window.c.o" \
"CMakeFiles/glfw.dir/xkb_unicode.c.o" \
"CMakeFiles/glfw.dir/linux_joystick.c.o" \
"CMakeFiles/glfw.dir/posix_time.c.o" \
"CMakeFiles/glfw.dir/posix_tls.c.o" \
"CMakeFiles/glfw.dir/glx_context.c.o"

# External object files for target glfw
glfw_EXTERNAL_OBJECTS =

/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build.make
/home/cg/cgue2/cglib/build/glfw/src/libglfw3.a: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cg/cgue2/02_whitted/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking C static library libglfw3.a"
	cd /home/cg/cgue2/cglib/build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean_target.cmake
	cd /home/cg/cgue2/cglib/build/glfw/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build: /home/cg/cgue2/cglib/build/glfw/src/libglfw3.a

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/build

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/context.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/init.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/input.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/monitor.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/window.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_init.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_monitor.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/x11_window.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/xkb_unicode.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/linux_joystick.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_time.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o.requires
/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires: /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/glx_context.c.o.requires

.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/requires

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/clean:
	cd /home/cg/cgue2/cglib/build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean.cmake
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/clean

/home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/depend:
	cd /home/cg/cgue2/02_whitted/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cg/cgue2/02_whitted /home/cg/cgue2/cglib/lib/glfw/src /home/cg/cgue2/02_whitted/build /home/cg/cgue2/cglib/build/glfw/src /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : /home/cg/cgue2/cglib/build/glfw/src/CMakeFiles/glfw.dir/depend

