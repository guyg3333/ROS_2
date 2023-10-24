cmake_minimum_required(VERSION 3.8)
project(video_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(tutorial_interfaces REQUIRED)                     

set(PUB_INC ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../Lib/gStreamLib ${CMAKE_BINARY_DIR}/bin)


add_executable(${PROJECT_NAME} src/publisher_member_function.cpp)
target_link_libraries(${PROJECT_NAME} Boost::thread gStreamLib)
target_include_directories(${PROJECT_NAME} PRIVATE ${PUB_INC})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs tutorial_interfaces)

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
