cmake_minimum_required(VERSION 3.10)
project(Kondo_KRS_RPi_GPIO)

# Set output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Include header files
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add the dynamic library
add_library(kondoKrsRpi SHARED 
src/IcsBaseClass.cpp 
src/IcsHardSerialClass.cpp)