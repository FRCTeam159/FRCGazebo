cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)

add_compile_definitions(M_PI=3.14159265358979323846)
add_compile_definitions(GZ_TEST=1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(NT_LIBS 
    $ENV{MY_GAZEBO}/wpilib/libs/ntcore.lib;
    $ENV{MY_GAZEBO}/wpilib/libs/wpiutil.lib;
)
include_directories(${GAZEBO_INCLUDE_DIRS} $ENV{MY_GAZEBO}/wpilib/includes)

link_directories(${GAZEBO_LIBRARY_DIRS} $ENV{MY_GAZEBO}/wpilib/libs)

add_executable(
    GzProgram 
../src/GzMain.cpp 
../src/GzMotor.cpp 
../src/GzNode.cpp 
../src/GzEncoder.cpp 
../src/GzGyro.cpp
../src/GzClock.cpp
)
target_link_libraries(GzProgram ${GAZEBO_LIBRARIES} ${NT_LIBS})


