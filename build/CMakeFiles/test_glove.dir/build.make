# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mel/arp/project/test-glove

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mel/arp/project/test-glove/build

# Include any dependencies generated for this target.
include CMakeFiles/test_glove.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_glove.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_glove.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_glove.dir/flags.make

CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o: CMakeFiles/test_glove.dir/flags.make
CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o: /home/mel/arp/project/test-glove/src/DataCaptureApp.cpp
CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o: CMakeFiles/test_glove.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mel/arp/project/test-glove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o -MF CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o.d -o CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o -c /home/mel/arp/project/test-glove/src/DataCaptureApp.cpp

CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mel/arp/project/test-glove/src/DataCaptureApp.cpp > CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.i

CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mel/arp/project/test-glove/src/DataCaptureApp.cpp -o CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.s

CMakeFiles/test_glove.dir/src/GloveData.cpp.o: CMakeFiles/test_glove.dir/flags.make
CMakeFiles/test_glove.dir/src/GloveData.cpp.o: /home/mel/arp/project/test-glove/src/GloveData.cpp
CMakeFiles/test_glove.dir/src/GloveData.cpp.o: CMakeFiles/test_glove.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mel/arp/project/test-glove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_glove.dir/src/GloveData.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_glove.dir/src/GloveData.cpp.o -MF CMakeFiles/test_glove.dir/src/GloveData.cpp.o.d -o CMakeFiles/test_glove.dir/src/GloveData.cpp.o -c /home/mel/arp/project/test-glove/src/GloveData.cpp

CMakeFiles/test_glove.dir/src/GloveData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_glove.dir/src/GloveData.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mel/arp/project/test-glove/src/GloveData.cpp > CMakeFiles/test_glove.dir/src/GloveData.cpp.i

CMakeFiles/test_glove.dir/src/GloveData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_glove.dir/src/GloveData.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mel/arp/project/test-glove/src/GloveData.cpp -o CMakeFiles/test_glove.dir/src/GloveData.cpp.s

CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o: CMakeFiles/test_glove.dir/flags.make
CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o: /home/mel/arp/project/test-glove/src/capture_pointcloud.cpp
CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o: CMakeFiles/test_glove.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mel/arp/project/test-glove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o -MF CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o.d -o CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o -c /home/mel/arp/project/test-glove/src/capture_pointcloud.cpp

CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mel/arp/project/test-glove/src/capture_pointcloud.cpp > CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.i

CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mel/arp/project/test-glove/src/capture_pointcloud.cpp -o CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.s

# Object files for target test_glove
test_glove_OBJECTS = \
"CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o" \
"CMakeFiles/test_glove.dir/src/GloveData.cpp.o" \
"CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o"

# External object files for target test_glove
test_glove_EXTERNAL_OBJECTS =

Debug/test_glove/test_glove: CMakeFiles/test_glove.dir/src/DataCaptureApp.cpp.o
Debug/test_glove/test_glove: CMakeFiles/test_glove.dir/src/GloveData.cpp.o
Debug/test_glove/test_glove: CMakeFiles/test_glove.dir/src/capture_pointcloud.cpp.o
Debug/test_glove/test_glove: CMakeFiles/test_glove.dir/build.make
Debug/test_glove/test_glove: /home/mel/Cinder/lib/linux/x86_64/ogl/Debug/libcinder.a
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libGL.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libGLU.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libSM.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libICE.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libX11.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libXext.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libz.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libcurl.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libfontconfig.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libpulse.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libmpg123.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libsndfile.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgobject-2.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libglib-2.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgstbase-1.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgstapp-1.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgstvideo-1.0.so
Debug/test_glove/test_glove: /usr/lib/x86_64-linux-gnu/libgstgl-1.0.so
Debug/test_glove/test_glove: CMakeFiles/test_glove.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mel/arp/project/test-glove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Debug/test_glove/test_glove"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_glove.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_glove.dir/build: Debug/test_glove/test_glove
.PHONY : CMakeFiles/test_glove.dir/build

CMakeFiles/test_glove.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_glove.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_glove.dir/clean

CMakeFiles/test_glove.dir/depend:
	cd /home/mel/arp/project/test-glove/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mel/arp/project/test-glove /home/mel/arp/project/test-glove /home/mel/arp/project/test-glove/build /home/mel/arp/project/test-glove/build /home/mel/arp/project/test-glove/build/CMakeFiles/test_glove.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_glove.dir/depend
