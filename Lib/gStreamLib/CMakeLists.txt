cmake_minimum_required(VERSION 3.0)
project(gStreamLib)

# Find the GStreamer package
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED 
    gstreamer-1.0
)

find_package(Boost REQUIRED COMPONENTS thread)


# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add include directories
#include_directories(${GST_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/)

# Add source files to the executable
add_library(gStreamLib ${CMAKE_CURRENT_SOURCE_DIR}/gStreamLib.cpp)

# Link the executable with GStreamer libraries
target_link_libraries(gStreamLib ${GST_LIBRARIES})
target_include_directories(gStreamLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${GST_INCLUDE_DIRS})

# Optional: Set output directory for the executable
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#add_subdirectory(ut)

# Optional: Add any other compiler flags or libraries you may need
# For example, to enable C++ exceptions:
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")