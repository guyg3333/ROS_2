cmake_minimum_required(VERSION 3.8)
project(video_sub_rec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


set(VIDEOSUB_INC ${CMAKE_CURRENT_SOURCE_DIR}/include )
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../Lib/gStreamLib ${CMAKE_BINARY_DIR}/bin)


add_executable(${PROJECT_NAME} src/subscriber_rec.cpp)
target_link_libraries(${PROJECT_NAME} Boost::thread gStreamLib)
ament_target_dependencies(${PROJECT_NAME} rclcpp tutorial_interfaces)
target_include_directories(${PROJECT_NAME} PRIVATE ${VIDEOSUB_INC})


install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
