cmake_minimum_required(VERSION 3.22)
project(tl_tcb710_config)

find_package(ament_cmake REQUIRED)

# tl_description is a pure Python package (no CMake config), needed only at runtime
# find_package(tl_description REQUIRED)  # ← removed: handled via exec_depend in package.xml

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
