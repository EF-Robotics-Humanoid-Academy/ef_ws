cmake_minimum_required(VERSION 3.5)
project(g1_moveit_29dof)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
