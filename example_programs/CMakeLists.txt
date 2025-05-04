cmake_minimum_required(VERSION 3.10)

# Set the project name
project(AllMotorsExample)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(all_motors src/all_motors.cpp)

# Include directories for kondoKrsRpi
target_include_directories(all_motors PUBLIC ${CMAKE_SOURCE_DIR}/../IcsClass_V210/include)


# Link libraries
target_link_libraries(all_motors
    wiringPi
    ${CMAKE_SOURCE_DIR}/../IcsClass_V210/lib/libkondoKrsRpi.so
)