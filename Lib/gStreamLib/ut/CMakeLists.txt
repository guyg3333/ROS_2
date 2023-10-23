cmake_minimum_required(VERSION 3.0)
project(utGst)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../gStreamLib ${EXECUTABLE_OUTPUT_PATH})
find_package(Boost REQUIRED COMPONENTS thread)


# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add source files to the executable
add_executable(utGst utGstLib.cpp)

# Link the executable with GStreamer libraries
target_link_libraries(utGst  Boost::thread gStreamLib)

# Optional: Set output directory for the executable
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})


# Optional: Add any other compiler flags or libraries you may need
# For example, to enable C++ exceptions:
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")