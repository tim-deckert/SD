# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/luish/Schoolz/SD/Test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luish/Schoolz/SD/Test/build

# Include any dependencies generated for this target.
include CMakeFiles/rewtMain.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rewtMain.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rewtMain.dir/flags.make

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o: CMakeFiles/rewtMain.dir/flags.make
CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o: ../src/rewtMain.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o -c /home/luish/Schoolz/SD/Test/src/rewtMain.cpp

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rewtMain.dir/src/rewtMain.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luish/Schoolz/SD/Test/src/rewtMain.cpp > CMakeFiles/rewtMain.dir/src/rewtMain.cpp.i

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rewtMain.dir/src/rewtMain.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luish/Schoolz/SD/Test/src/rewtMain.cpp -o CMakeFiles/rewtMain.dir/src/rewtMain.cpp.s

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.requires:

.PHONY : CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.requires

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.provides: CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.requires
	$(MAKE) -f CMakeFiles/rewtMain.dir/build.make CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.provides.build
.PHONY : CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.provides

CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.provides.build: CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o


CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o: CMakeFiles/rewtMain.dir/flags.make
CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o: ../src/CloudVisualizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o -c /home/luish/Schoolz/SD/Test/src/CloudVisualizer.cpp

CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luish/Schoolz/SD/Test/src/CloudVisualizer.cpp > CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.i

CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luish/Schoolz/SD/Test/src/CloudVisualizer.cpp -o CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.s

CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.requires:

.PHONY : CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.requires

CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.provides: CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/rewtMain.dir/build.make CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.provides.build
.PHONY : CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.provides

CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.provides.build: CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o


CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o: CMakeFiles/rewtMain.dir/flags.make
CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o: ../src/random_sample_consensus.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o -c /home/luish/Schoolz/SD/Test/src/random_sample_consensus.cpp

CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luish/Schoolz/SD/Test/src/random_sample_consensus.cpp > CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.i

CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luish/Schoolz/SD/Test/src/random_sample_consensus.cpp -o CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.s

CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.requires:

.PHONY : CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.requires

CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.provides: CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.requires
	$(MAKE) -f CMakeFiles/rewtMain.dir/build.make CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.provides.build
.PHONY : CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.provides

CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.provides.build: CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o


CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o: CMakeFiles/rewtMain.dir/flags.make
CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o: ../src/voxel_grid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o -c /home/luish/Schoolz/SD/Test/src/voxel_grid.cpp

CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luish/Schoolz/SD/Test/src/voxel_grid.cpp > CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.i

CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luish/Schoolz/SD/Test/src/voxel_grid.cpp -o CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.s

CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.requires:

.PHONY : CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.requires

CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.provides: CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.requires
	$(MAKE) -f CMakeFiles/rewtMain.dir/build.make CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.provides.build
.PHONY : CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.provides

CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.provides.build: CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o


CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o: CMakeFiles/rewtMain.dir/flags.make
CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o: ../src/load_pcd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o -c /home/luish/Schoolz/SD/Test/src/load_pcd.cpp

CMakeFiles/rewtMain.dir/src/load_pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rewtMain.dir/src/load_pcd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luish/Schoolz/SD/Test/src/load_pcd.cpp > CMakeFiles/rewtMain.dir/src/load_pcd.cpp.i

CMakeFiles/rewtMain.dir/src/load_pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rewtMain.dir/src/load_pcd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luish/Schoolz/SD/Test/src/load_pcd.cpp -o CMakeFiles/rewtMain.dir/src/load_pcd.cpp.s

CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.requires:

.PHONY : CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.requires

CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.provides: CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.requires
	$(MAKE) -f CMakeFiles/rewtMain.dir/build.make CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.provides.build
.PHONY : CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.provides

CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.provides.build: CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o


# Object files for target rewtMain
rewtMain_OBJECTS = \
"CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o" \
"CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o" \
"CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o" \
"CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o" \
"CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o"

# External object files for target rewtMain
rewtMain_EXTERNAL_OBJECTS =

rewtMain: CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o
rewtMain: CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o
rewtMain: CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o
rewtMain: CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o
rewtMain: CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o
rewtMain: CMakeFiles/rewtMain.dir/build.make
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_system.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_regex.so
rewtMain: /usr/local/lib/libpcl_common.so
rewtMain: /usr/local/lib/libpcl_octree.so
rewtMain: /home/luish/OpenNI-Linux-x64-2.3/Redist/libOpenNI2.so
rewtMain: /usr/local/lib/libpcl_io.so
rewtMain: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
rewtMain: /usr/local/lib/libpcl_kdtree.so
rewtMain: /usr/local/lib/libpcl_search.so
rewtMain: /usr/local/lib/libpcl_sample_consensus.so
rewtMain: /usr/local/lib/libpcl_filters.so
rewtMain: /usr/local/lib/libpcl_features.so
rewtMain: /usr/local/lib/libpcl_ml.so
rewtMain: /usr/local/lib/libpcl_segmentation.so
rewtMain: /usr/local/lib/libpcl_visualization.so
rewtMain: /usr/lib/x86_64-linux-gnu/libqhull.so
rewtMain: /usr/local/lib/libpcl_surface.so
rewtMain: /usr/local/lib/libpcl_registration.so
rewtMain: /usr/local/lib/libpcl_keypoints.so
rewtMain: /usr/local/lib/libpcl_tracking.so
rewtMain: /usr/local/lib/libpcl_recognition.so
rewtMain: /usr/local/lib/libpcl_stereo.so
rewtMain: /usr/local/lib/libpcl_outofcore.so
rewtMain: /usr/local/lib/libpcl_people.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_system.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
rewtMain: /usr/lib/x86_64-linux-gnu/libboost_regex.so
rewtMain: /usr/lib/x86_64-linux-gnu/libqhull.so
rewtMain: /home/luish/OpenNI-Linux-x64-2.3/Redist/libOpenNI2.so
rewtMain: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
rewtMain: /usr/lib/libvtkGenericFiltering.so.5.10.1
rewtMain: /usr/lib/libvtkGeovis.so.5.10.1
rewtMain: /usr/lib/libvtkCharts.so.5.10.1
rewtMain: /usr/local/lib/libpcl_common.so
rewtMain: /usr/local/lib/libpcl_octree.so
rewtMain: /usr/local/lib/libpcl_io.so
rewtMain: /usr/local/lib/libpcl_kdtree.so
rewtMain: /usr/local/lib/libpcl_search.so
rewtMain: /usr/local/lib/libpcl_sample_consensus.so
rewtMain: /usr/local/lib/libpcl_filters.so
rewtMain: /usr/local/lib/libpcl_features.so
rewtMain: /usr/local/lib/libpcl_ml.so
rewtMain: /usr/local/lib/libpcl_segmentation.so
rewtMain: /usr/local/lib/libpcl_visualization.so
rewtMain: /usr/local/lib/libpcl_surface.so
rewtMain: /usr/local/lib/libpcl_registration.so
rewtMain: /usr/local/lib/libpcl_keypoints.so
rewtMain: /usr/local/lib/libpcl_tracking.so
rewtMain: /usr/local/lib/libpcl_recognition.so
rewtMain: /usr/local/lib/libpcl_stereo.so
rewtMain: /usr/local/lib/libpcl_outofcore.so
rewtMain: /usr/local/lib/libpcl_people.so
rewtMain: /usr/lib/libvtkViews.so.5.10.1
rewtMain: /usr/lib/libvtkInfovis.so.5.10.1
rewtMain: /usr/lib/libvtkWidgets.so.5.10.1
rewtMain: /usr/lib/libvtkVolumeRendering.so.5.10.1
rewtMain: /usr/lib/libvtkHybrid.so.5.10.1
rewtMain: /usr/lib/libvtkParallel.so.5.10.1
rewtMain: /usr/lib/libvtkRendering.so.5.10.1
rewtMain: /usr/lib/libvtkImaging.so.5.10.1
rewtMain: /usr/lib/libvtkGraphics.so.5.10.1
rewtMain: /usr/lib/libvtkIO.so.5.10.1
rewtMain: /usr/lib/libvtkFiltering.so.5.10.1
rewtMain: /usr/lib/libvtkCommon.so.5.10.1
rewtMain: /usr/lib/libvtksys.so.5.10.1
rewtMain: CMakeFiles/rewtMain.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luish/Schoolz/SD/Test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable rewtMain"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rewtMain.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rewtMain.dir/build: rewtMain

.PHONY : CMakeFiles/rewtMain.dir/build

CMakeFiles/rewtMain.dir/requires: CMakeFiles/rewtMain.dir/src/rewtMain.cpp.o.requires
CMakeFiles/rewtMain.dir/requires: CMakeFiles/rewtMain.dir/src/CloudVisualizer.cpp.o.requires
CMakeFiles/rewtMain.dir/requires: CMakeFiles/rewtMain.dir/src/random_sample_consensus.cpp.o.requires
CMakeFiles/rewtMain.dir/requires: CMakeFiles/rewtMain.dir/src/voxel_grid.cpp.o.requires
CMakeFiles/rewtMain.dir/requires: CMakeFiles/rewtMain.dir/src/load_pcd.cpp.o.requires

.PHONY : CMakeFiles/rewtMain.dir/requires

CMakeFiles/rewtMain.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rewtMain.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rewtMain.dir/clean

CMakeFiles/rewtMain.dir/depend:
	cd /home/luish/Schoolz/SD/Test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luish/Schoolz/SD/Test /home/luish/Schoolz/SD/Test /home/luish/Schoolz/SD/Test/build /home/luish/Schoolz/SD/Test/build /home/luish/Schoolz/SD/Test/build/CMakeFiles/rewtMain.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rewtMain.dir/depend

