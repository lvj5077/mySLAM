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
CMAKE_SOURCE_DIR = /Users/lingqiujin/Q_MAC/work/mySLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/lingqiujin/Q_MAC/work/mySLAM/build

# Include any dependencies generated for this target.
include src/CMakeFiles/slamBase.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/slamBase.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/slamBase.dir/flags.make

src/CMakeFiles/slamBase.dir/slamBase.cpp.o: src/CMakeFiles/slamBase.dir/flags.make
src/CMakeFiles/slamBase.dir/slamBase.cpp.o: ../src/slamBase.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lingqiujin/Q_MAC/work/mySLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/slamBase.dir/slamBase.cpp.o"
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slamBase.dir/slamBase.cpp.o -c /Users/lingqiujin/Q_MAC/work/mySLAM/src/slamBase.cpp

src/CMakeFiles/slamBase.dir/slamBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slamBase.dir/slamBase.cpp.i"
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lingqiujin/Q_MAC/work/mySLAM/src/slamBase.cpp > CMakeFiles/slamBase.dir/slamBase.cpp.i

src/CMakeFiles/slamBase.dir/slamBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slamBase.dir/slamBase.cpp.s"
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lingqiujin/Q_MAC/work/mySLAM/src/slamBase.cpp -o CMakeFiles/slamBase.dir/slamBase.cpp.s

# Object files for target slamBase
slamBase_OBJECTS = \
"CMakeFiles/slamBase.dir/slamBase.cpp.o"

# External object files for target slamBase
slamBase_EXTERNAL_OBJECTS =

../lib/libslamBase.a: src/CMakeFiles/slamBase.dir/slamBase.cpp.o
../lib/libslamBase.a: src/CMakeFiles/slamBase.dir/build.make
../lib/libslamBase.a: src/CMakeFiles/slamBase.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/lingqiujin/Q_MAC/work/mySLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../lib/libslamBase.a"
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && $(CMAKE_COMMAND) -P CMakeFiles/slamBase.dir/cmake_clean_target.cmake
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slamBase.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/slamBase.dir/build: ../lib/libslamBase.a

.PHONY : src/CMakeFiles/slamBase.dir/build

src/CMakeFiles/slamBase.dir/clean:
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build/src && $(CMAKE_COMMAND) -P CMakeFiles/slamBase.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/slamBase.dir/clean

src/CMakeFiles/slamBase.dir/depend:
	cd /Users/lingqiujin/Q_MAC/work/mySLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lingqiujin/Q_MAC/work/mySLAM /Users/lingqiujin/Q_MAC/work/mySLAM/src /Users/lingqiujin/Q_MAC/work/mySLAM/build /Users/lingqiujin/Q_MAC/work/mySLAM/build/src /Users/lingqiujin/Q_MAC/work/mySLAM/build/src/CMakeFiles/slamBase.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/slamBase.dir/depend

