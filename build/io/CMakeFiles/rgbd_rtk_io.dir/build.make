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
CMAKE_SOURCE_DIR = /home/hmarcos/rgbd_rtk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hmarcos/rgbd_rtk/build

# Include any dependencies generated for this target.
include io/CMakeFiles/rgbd_rtk_io.dir/depend.make

# Include the progress variables for this target.
include io/CMakeFiles/rgbd_rtk_io.dir/progress.make

# Include the compile flags for this target's objects.
include io/CMakeFiles/rgbd_rtk_io.dir/flags.make

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o: io/CMakeFiles/rgbd_rtk_io.dir/flags.make
io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o: ../io/sequence_loader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hmarcos/rgbd_rtk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o -c /home/hmarcos/rgbd_rtk/io/sequence_loader.cpp

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.i"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hmarcos/rgbd_rtk/io/sequence_loader.cpp > CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.i

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.s"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hmarcos/rgbd_rtk/io/sequence_loader.cpp -o CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.s

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.requires:

.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.requires

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.provides: io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.requires
	$(MAKE) -f io/CMakeFiles/rgbd_rtk_io.dir/build.make io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.provides.build
.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.provides

io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.provides.build: io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o


io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o: io/CMakeFiles/rgbd_rtk_io.dir/flags.make
io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o: ../io/rgbd_loader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hmarcos/rgbd_rtk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o -c /home/hmarcos/rgbd_rtk/io/rgbd_loader.cpp

io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.i"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hmarcos/rgbd_rtk/io/rgbd_loader.cpp > CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.i

io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.s"
	cd /home/hmarcos/rgbd_rtk/build/io && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hmarcos/rgbd_rtk/io/rgbd_loader.cpp -o CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.s

io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.requires:

.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.requires

io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.provides: io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.requires
	$(MAKE) -f io/CMakeFiles/rgbd_rtk_io.dir/build.make io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.provides.build
.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.provides

io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.provides.build: io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o


# Object files for target rgbd_rtk_io
rgbd_rtk_io_OBJECTS = \
"CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o" \
"CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o"

# External object files for target rgbd_rtk_io
rgbd_rtk_io_EXTERNAL_OBJECTS =

io/lib/librgbd_rtk_io.so.1.0: io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o
io/lib/librgbd_rtk_io.so.1.0: io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o
io/lib/librgbd_rtk_io.so.1.0: io/CMakeFiles/rgbd_rtk_io.dir/build.make
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_stitching.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_superres.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_videostab.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_aruco.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_bgsegm.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_bioinspired.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_ccalib.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_cvv.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_dpm.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_face.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_freetype.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_fuzzy.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_hdf.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_img_hash.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_line_descriptor.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_optflow.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_reg.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_rgbd.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_saliency.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_sfm.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_stereo.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_structured_light.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_surface_matching.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_tracking.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_xfeatures2d.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_ximgproc.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_xobjdetect.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_xphoto.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_photo.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_shape.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_calib3d.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_phase_unwrapping.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_video.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_datasets.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_plot.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_text.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_dnn.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_features2d.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_flann.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_highgui.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_ml.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_videoio.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_imgcodecs.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_objdetect.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_imgproc.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: /usr/local/lib/libopencv_core.so.3.3.1
io/lib/librgbd_rtk_io.so.1.0: io/CMakeFiles/rgbd_rtk_io.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hmarcos/rgbd_rtk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library lib/librgbd_rtk_io.so"
	cd /home/hmarcos/rgbd_rtk/build/io && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd_rtk_io.dir/link.txt --verbose=$(VERBOSE)
	cd /home/hmarcos/rgbd_rtk/build/io && $(CMAKE_COMMAND) -E cmake_symlink_library lib/librgbd_rtk_io.so.1.0 lib/librgbd_rtk_io.so.1.0 lib/librgbd_rtk_io.so

io/lib/librgbd_rtk_io.so: io/lib/librgbd_rtk_io.so.1.0
	@$(CMAKE_COMMAND) -E touch_nocreate io/lib/librgbd_rtk_io.so

# Rule to build all files generated by this target.
io/CMakeFiles/rgbd_rtk_io.dir/build: io/lib/librgbd_rtk_io.so

.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/build

io/CMakeFiles/rgbd_rtk_io.dir/requires: io/CMakeFiles/rgbd_rtk_io.dir/sequence_loader.cpp.o.requires
io/CMakeFiles/rgbd_rtk_io.dir/requires: io/CMakeFiles/rgbd_rtk_io.dir/rgbd_loader.cpp.o.requires

.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/requires

io/CMakeFiles/rgbd_rtk_io.dir/clean:
	cd /home/hmarcos/rgbd_rtk/build/io && $(CMAKE_COMMAND) -P CMakeFiles/rgbd_rtk_io.dir/cmake_clean.cmake
.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/clean

io/CMakeFiles/rgbd_rtk_io.dir/depend:
	cd /home/hmarcos/rgbd_rtk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hmarcos/rgbd_rtk /home/hmarcos/rgbd_rtk/io /home/hmarcos/rgbd_rtk/build /home/hmarcos/rgbd_rtk/build/io /home/hmarcos/rgbd_rtk/build/io/CMakeFiles/rgbd_rtk_io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : io/CMakeFiles/rgbd_rtk_io.dir/depend

