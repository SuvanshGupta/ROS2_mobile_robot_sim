cmake_minimum_required(VERSION 3.8)
project(my_urdf_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install URDF, launch, rviz, world, and meshes
install(
  DIRECTORY urdf launch rviz world meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules and pid_controller scripts
install(
  DIRECTORY pid_controller/
  DESTINATION lib/${PROJECT_NAME}/pid_controller
)

# Install the Python executable (pid_controller.py)
install(
  PROGRAMS pid_controller/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS pid_controller/pid_controller1.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS pid_controller/pid_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS pid_controller/pid_controller2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS pid_controller/pid_updated.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS pid_controller/SFOA.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS pid_controller/HHO.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS pid_controller/pid_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

