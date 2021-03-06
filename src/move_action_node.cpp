// Copyright 2022 L4ROS2

#include "MoveAction.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>("move_to_location", 500ms);

  node->set_parameter(rclcpp::Parameter("action_name", "move_to_location"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
