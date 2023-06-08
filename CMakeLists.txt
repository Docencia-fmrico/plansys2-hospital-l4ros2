cmake_minimum_required(VERSION 3.5)
project(plansys2_hospital_l4ros2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

set(CMAKE_CXX_STANDARD 17)


set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    geometry_msgs
    tf2_geometry_msgs
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED src/MoveAction.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
target_link_libraries(move_action_node ${PROJECT_NAME})

add_executable(move_through_door_action_node src/move_through_door_action_node.cpp)
ament_target_dependencies(move_through_door_action_node ${dependencies})
target_link_libraries(move_through_door_action_node ${PROJECT_NAME})

add_executable(move_near_door_action_node src/move_near_door_action_node.cpp)
ament_target_dependencies(move_near_door_action_node ${dependencies})
target_link_libraries(move_near_door_action_node ${PROJECT_NAME})

add_executable(hospital_controller_node src/hospital_controller_node.cpp)
ament_target_dependencies(hospital_controller_node ${dependencies})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

add_executable(pick_action_node src/pick_action_node.cpp)
ament_target_dependencies(pick_action_node ${dependencies})

add_executable(drop_action_node src/drop_action_node.cpp)
ament_target_dependencies(drop_action_node ${dependencies})

add_executable(close_action_node src/close_action_node.cpp)
ament_target_dependencies(close_action_node ${dependencies})

add_executable(open_action_node src/open_action_node.cpp)
ament_target_dependencies(open_action_node ${dependencies})


install(DIRECTORY launch pddl config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  move_through_door_action_node
  move_near_door_action_node
  pick_action_node
  drop_action_node
  close_action_node
  open_action_node
  hospital_controller_node
  nav2_sim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
