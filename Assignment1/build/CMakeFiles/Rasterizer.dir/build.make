# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.21

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = I:\CMake\bin\cmake.exe

# The command to remove a file.
RM = I:\CMake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = I:\GamesRender101\Games_Render\Assignment1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = I:\GamesRender101\Games_Render\Assignment1\build

# Include any dependencies generated for this target.
include CMakeFiles/Rasterizer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Rasterizer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Rasterizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rasterizer.dir/flags.make

CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/main.cpp.obj: ../main.cpp
CMakeFiles/Rasterizer.dir/main.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=I:\GamesRender101\Games_Render\Assignment1\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rasterizer.dir/main.cpp.obj"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/main.cpp.obj -MF CMakeFiles\Rasterizer.dir\main.cpp.obj.d -o CMakeFiles\Rasterizer.dir\main.cpp.obj -c I:\GamesRender101\Games_Render\Assignment1\main.cpp

CMakeFiles/Rasterizer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/main.cpp.i"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E I:\GamesRender101\Games_Render\Assignment1\main.cpp > CMakeFiles\Rasterizer.dir\main.cpp.i

CMakeFiles/Rasterizer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/main.cpp.s"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S I:\GamesRender101\Games_Render\Assignment1\main.cpp -o CMakeFiles\Rasterizer.dir\main.cpp.s

CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: ../rasterizer.cpp
CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=I:\GamesRender101\Games_Render\Assignment1\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj -MF CMakeFiles\Rasterizer.dir\rasterizer.cpp.obj.d -o CMakeFiles\Rasterizer.dir\rasterizer.cpp.obj -c I:\GamesRender101\Games_Render\Assignment1\rasterizer.cpp

CMakeFiles/Rasterizer.dir/rasterizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/rasterizer.cpp.i"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E I:\GamesRender101\Games_Render\Assignment1\rasterizer.cpp > CMakeFiles\Rasterizer.dir\rasterizer.cpp.i

CMakeFiles/Rasterizer.dir/rasterizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/rasterizer.cpp.s"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S I:\GamesRender101\Games_Render\Assignment1\rasterizer.cpp -o CMakeFiles\Rasterizer.dir\rasterizer.cpp.s

CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/includes_CXX.rsp
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: ../Triangle.cpp
CMakeFiles/Rasterizer.dir/Triangle.cpp.obj: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=I:\GamesRender101\Games_Render\Assignment1\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Rasterizer.dir/Triangle.cpp.obj"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/Triangle.cpp.obj -MF CMakeFiles\Rasterizer.dir\Triangle.cpp.obj.d -o CMakeFiles\Rasterizer.dir\Triangle.cpp.obj -c I:\GamesRender101\Games_Render\Assignment1\Triangle.cpp

CMakeFiles/Rasterizer.dir/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/Triangle.cpp.i"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E I:\GamesRender101\Games_Render\Assignment1\Triangle.cpp > CMakeFiles\Rasterizer.dir\Triangle.cpp.i

CMakeFiles/Rasterizer.dir/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/Triangle.cpp.s"
	I:\mw64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S I:\GamesRender101\Games_Render\Assignment1\Triangle.cpp -o CMakeFiles\Rasterizer.dir\Triangle.cpp.s

# Object files for target Rasterizer
Rasterizer_OBJECTS = \
"CMakeFiles/Rasterizer.dir/main.cpp.obj" \
"CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj" \
"CMakeFiles/Rasterizer.dir/Triangle.cpp.obj"

# External object files for target Rasterizer
Rasterizer_EXTERNAL_OBJECTS =

Rasterizer.exe: CMakeFiles/Rasterizer.dir/main.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/rasterizer.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/Triangle.cpp.obj
Rasterizer.exe: CMakeFiles/Rasterizer.dir/build.make
Rasterizer.exe: I:/opencv/build/lib/libopencv_gapi451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_highgui451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_ml451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_objdetect451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_photo451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_stitching451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_video451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_videoio451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_dnn451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_imgcodecs451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_calib3d451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_features2d451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_flann451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_imgproc451.dll.a
Rasterizer.exe: I:/opencv/build/lib/libopencv_core451.dll.a
Rasterizer.exe: CMakeFiles/Rasterizer.dir/linklibs.rsp
Rasterizer.exe: CMakeFiles/Rasterizer.dir/objects1.rsp
Rasterizer.exe: CMakeFiles/Rasterizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=I:\GamesRender101\Games_Render\Assignment1\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Rasterizer.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Rasterizer.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rasterizer.dir/build: Rasterizer.exe
.PHONY : CMakeFiles/Rasterizer.dir/build

CMakeFiles/Rasterizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Rasterizer.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Rasterizer.dir/clean

CMakeFiles/Rasterizer.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" I:\GamesRender101\Games_Render\Assignment1 I:\GamesRender101\Games_Render\Assignment1 I:\GamesRender101\Games_Render\Assignment1\build I:\GamesRender101\Games_Render\Assignment1\build I:\GamesRender101\Games_Render\Assignment1\build\CMakeFiles\Rasterizer.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Rasterizer.dir/depend

