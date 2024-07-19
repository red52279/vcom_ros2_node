cmake_minimum_required(VERSION 3.8)

project(vcom_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vcom_interfaces REQUIRED)



add_executable(vcom_test_receive src/vcom_receive.cpp)
ament_target_dependencies(vcom_test_receive rclcpp vcom_interfaces)

add_executable(vcom_test_send src/vcom_send.cpp)
ament_target_dependencies(vcom_test_send rclcpp vcom_interfaces)


target_include_directories(vcom_test_send PUBLIC include)

install(TARGETS
  	vcom_test_send
  	vcom_test_receive
	DESTINATION lib/${PROJECT_NAME})

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
