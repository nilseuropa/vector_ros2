cmake_minimum_required(VERSION 3.8)
project(vector_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlayAnimationTrigger.srv"
  "srv/SayText.srv"
)

install(DIRECTORY ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vector_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
