# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/lingqiujin/Desktop/myVOsum

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/lingqiujin/Desktop/myVOsum/build

# Include any dependencies generated for this target.
include src/CMakeFiles/testT.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/testT.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/testT.dir/flags.make

src/CMakeFiles/testT.dir/testT.cpp.o: src/CMakeFiles/testT.dir/flags.make
src/CMakeFiles/testT.dir/testT.cpp.o: ../src/testT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lingqiujin/Desktop/myVOsum/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/testT.dir/testT.cpp.o"
	cd /Users/lingqiujin/Desktop/myVOsum/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testT.dir/testT.cpp.o -c /Users/lingqiujin/Desktop/myVOsum/src/testT.cpp

src/CMakeFiles/testT.dir/testT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testT.dir/testT.cpp.i"
	cd /Users/lingqiujin/Desktop/myVOsum/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lingqiujin/Desktop/myVOsum/src/testT.cpp > CMakeFiles/testT.dir/testT.cpp.i

src/CMakeFiles/testT.dir/testT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testT.dir/testT.cpp.s"
	cd /Users/lingqiujin/Desktop/myVOsum/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lingqiujin/Desktop/myVOsum/src/testT.cpp -o CMakeFiles/testT.dir/testT.cpp.s

# Object files for target testT
testT_OBJECTS = \
"CMakeFiles/testT.dir/testT.cpp.o"

# External object files for target testT
testT_EXTERNAL_OBJECTS =

../bin/testT: src/CMakeFiles/testT.dir/testT.cpp.o
../bin/testT: src/CMakeFiles/testT.dir/build.make
../bin/testT: ../lib/libslamBase.a
../bin/testT: ../lib/libpose_estimation.a
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_dnn.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_ml.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_objdetect.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_shape.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_stitching.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_superres.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_videostab.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_viz.3.4.5.dylib
../bin/testT: /usr/local/Cellar/opencv@3/3.4.5_1/lib/libopencv_xfeatures2d.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_calib3d.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_features2d.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_flann.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_highgui.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_photo.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_video.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_videoio.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_imgcodecs.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_imgproc.3.4.5.dylib
../bin/testT: /Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build/lib/libopencv_core.3.4.5.dylib
../bin/testT: ../lib/libmyG2Oedge.a
../bin/testT: src/CMakeFiles/testT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/lingqiujin/Desktop/myVOsum/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/testT"
	cd /Users/lingqiujin/Desktop/myVOsum/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/testT.dir/build: ../bin/testT

.PHONY : src/CMakeFiles/testT.dir/build

src/CMakeFiles/testT.dir/clean:
	cd /Users/lingqiujin/Desktop/myVOsum/build/src && $(CMAKE_COMMAND) -P CMakeFiles/testT.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/testT.dir/clean

src/CMakeFiles/testT.dir/depend:
	cd /Users/lingqiujin/Desktop/myVOsum/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lingqiujin/Desktop/myVOsum /Users/lingqiujin/Desktop/myVOsum/src /Users/lingqiujin/Desktop/myVOsum/build /Users/lingqiujin/Desktop/myVOsum/build/src /Users/lingqiujin/Desktop/myVOsum/build/src/CMakeFiles/testT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/testT.dir/depend

