# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/hjk/Downloads/CLion-2020.3.2/clion-2020.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/hjk/Downloads/CLion-2020.3.2/clion-2020.3.2/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hjk/swarm_ws/src/camera_detect_offb

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/usbcam_aruco_detect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/usbcam_aruco_detect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/usbcam_aruco_detect.dir/flags.make

CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o: CMakeFiles/usbcam_aruco_detect.dir/flags.make
CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o: ../src/usbcam_aruco_detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o -c /home/hjk/swarm_ws/src/camera_detect_offb/src/usbcam_aruco_detect.cpp

CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/swarm_ws/src/camera_detect_offb/src/usbcam_aruco_detect.cpp > CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.i

CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/swarm_ws/src/camera_detect_offb/src/usbcam_aruco_detect.cpp -o CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.s

# Object files for target usbcam_aruco_detect
usbcam_aruco_detect_OBJECTS = \
"CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o"

# External object files for target usbcam_aruco_detect
usbcam_aruco_detect_EXTERNAL_OBJECTS =

devel/lib/camera_detect_offb/usbcam_aruco_detect: CMakeFiles/usbcam_aruco_detect.dir/src/usbcam_aruco_detect.cpp.o
devel/lib/camera_detect_offb/usbcam_aruco_detect: CMakeFiles/usbcam_aruco_detect.dir/build.make
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libtf.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libtf2_ros.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libactionlib.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libtf2.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/libPocoFoundation.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libroscpp.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/librosconsole.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libroslib.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/librospack.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/librostime.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/camera_detect_offb/usbcam_aruco_detect: CMakeFiles/usbcam_aruco_detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/camera_detect_offb/usbcam_aruco_detect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usbcam_aruco_detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/usbcam_aruco_detect.dir/build: devel/lib/camera_detect_offb/usbcam_aruco_detect

.PHONY : CMakeFiles/usbcam_aruco_detect.dir/build

CMakeFiles/usbcam_aruco_detect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/usbcam_aruco_detect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/usbcam_aruco_detect.dir/clean

CMakeFiles/usbcam_aruco_detect.dir/depend:
	cd /home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hjk/swarm_ws/src/camera_detect_offb /home/hjk/swarm_ws/src/camera_detect_offb /home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug /home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug /home/hjk/swarm_ws/src/camera_detect_offb/cmake-build-debug/CMakeFiles/usbcam_aruco_detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/usbcam_aruco_detect.dir/depend

