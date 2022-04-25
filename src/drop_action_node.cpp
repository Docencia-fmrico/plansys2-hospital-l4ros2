// Copyright 2022 L4ROS2

#include <memory>

#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DropAction : public plansys2::ActionExecutorClient
{
public:
  DropAction()
  : plansys2::ActionExecutorClient("drop", 500ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.1;
      send_feedback(progress_, "drop running");
    } else {
      finish(true, 1.0, "Object was placed in the floor");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::string object_name = get_arguments()[1].c_str();
    std::cout << "Dropping object " << object_name << " [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drop"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
