# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/csnesummer/Documents/realsense/capture/cpp_stream

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/csnesummer/Documents/realsense/capture/cpp_stream/build

# Include any dependencies generated for this target.
include CMakeFiles/capture.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/capture.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/capture.dir/flags.make

CMakeFiles/capture.dir/capture.cpp.o: CMakeFiles/capture.dir/flags.make
CMakeFiles/capture.dir/capture.cpp.o: ../capture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/csnesummer/Documents/realsense/capture/cpp_stream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/capture.dir/capture.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/capture.dir/capture.cpp.o -c /home/csnesummer/Documents/realsense/capture/cpp_stream/capture.cpp

CMakeFiles/capture.dir/capture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/capture.dir/capture.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/csnesummer/Documents/realsense/capture/cpp_stream/capture.cpp > CMakeFiles/capture.dir/capture.cpp.i

CMakeFiles/capture.dir/capture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/capture.dir/capture.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/csnesummer/Documents/realsense/capture/cpp_stream/capture.cpp -o CMakeFiles/capture.dir/capture.cpp.s

CMakeFiles/capture.dir/capture.cpp.o.requires:

.PHONY : CMakeFiles/capture.dir/capture.cpp.o.requires

CMakeFiles/capture.dir/capture.cpp.o.provides: CMakeFiles/capture.dir/capture.cpp.o.requires
	$(MAKE) -f CMakeFiles/capture.dir/build.make CMakeFiles/capture.dir/capture.cpp.o.provides.build
.PHONY : CMakeFiles/capture.dir/capture.cpp.o.provides

CMakeFiles/capture.dir/capture.cpp.o.provides.build: CMakeFiles/capture.dir/capture.cpp.o


# Object files for target capture
capture_OBJECTS = \
"CMakeFiles/capture.dir/capture.cpp.o"

# External object files for target capture
capture_EXTERNAL_OBJECTS =

capture: CMakeFiles/capture.dir/capture.cpp.o
capture: CMakeFiles/capture.dir/build.make
capture: /usr/local/lib/libopencv_xphoto.so.3.1.0
capture: /usr/local/lib/libopencv_xobjdetect.so.3.1.0
capture: /usr/local/lib/libopencv_tracking.so.3.1.0
capture: /usr/local/lib/libopencv_surface_matching.so.3.1.0
capture: /usr/local/lib/libopencv_structured_light.so.3.1.0
capture: /usr/local/lib/libopencv_stereo.so.3.1.0
capture: /usr/local/lib/libopencv_saliency.so.3.1.0
capture: /usr/local/lib/libopencv_rgbd.so.3.1.0
capture: /usr/local/lib/libopencv_reg.so.3.1.0
capture: /usr/local/lib/libopencv_plot.so.3.1.0
capture: /usr/local/lib/libopencv_optflow.so.3.1.0
capture: /usr/local/lib/libopencv_line_descriptor.so.3.1.0
capture: /usr/local/lib/libopencv_hdf.so.3.1.0
capture: /usr/local/lib/libopencv_fuzzy.so.3.1.0
capture: /usr/local/lib/libopencv_dpm.so.3.1.0
capture: /usr/local/lib/libopencv_dnn.so.3.1.0
capture: /usr/local/lib/libopencv_datasets.so.3.1.0
capture: /usr/local/lib/libopencv_ccalib.so.3.1.0
capture: /usr/local/lib/libopencv_bioinspired.so.3.1.0
capture: /usr/local/lib/libopencv_bgsegm.so.3.1.0
capture: /usr/local/lib/libopencv_aruco.so.3.1.0
capture: /usr/local/lib/libopencv_viz.so.3.1.0
capture: /usr/local/lib/libopencv_videostab.so.3.1.0
capture: /usr/local/lib/libopencv_superres.so.3.1.0
capture: /usr/local/lib/libopencv_stitching.so.3.1.0
capture: /usr/local/lib/libopencv_photo.so.3.1.0
capture: /usr/local/lib/libopencv_text.so.3.1.0
capture: /usr/local/lib/libopencv_face.so.3.1.0
capture: /usr/local/lib/libopencv_ximgproc.so.3.1.0
capture: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
capture: /usr/local/lib/libopencv_shape.so.3.1.0
capture: /usr/local/lib/libopencv_video.so.3.1.0
capture: /usr/local/lib/libopencv_objdetect.so.3.1.0
capture: /usr/local/lib/libopencv_calib3d.so.3.1.0
capture: /usr/local/lib/libopencv_features2d.so.3.1.0
capture: /usr/local/lib/libopencv_ml.so.3.1.0
capture: /usr/local/lib/libopencv_highgui.so.3.1.0
capture: /usr/local/lib/libopencv_videoio.so.3.1.0
capture: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
capture: /usr/local/lib/libopencv_imgproc.so.3.1.0
capture: /usr/local/lib/libopencv_flann.so.3.1.0
capture: /usr/local/lib/libopencv_core.so.3.1.0
capture: CMakeFiles/capture.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/csnesummer/Documents/realsense/capture/cpp_stream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable capture"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/capture.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/capture.dir/build: capture

.PHONY : CMakeFiles/capture.dir/build

CMakeFiles/capture.dir/requires: CMakeFiles/capture.dir/capture.cpp.o.requires

.PHONY : CMakeFiles/capture.dir/requires

CMakeFiles/capture.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/capture.dir/cmake_clean.cmake
.PHONY : CMakeFiles/capture.dir/clean

CMakeFiles/capture.dir/depend:
	cd /home/csnesummer/Documents/realsense/capture/cpp_stream/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/csnesummer/Documents/realsense/capture/cpp_stream /home/csnesummer/Documents/realsense/capture/cpp_stream /home/csnesummer/Documents/realsense/capture/cpp_stream/build /home/csnesummer/Documents/realsense/capture/cpp_stream/build /home/csnesummer/Documents/realsense/capture/cpp_stream/build/CMakeFiles/capture.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/capture.dir/depend
