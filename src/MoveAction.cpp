// Copyright 2022 L4ROS2

#include "MoveAction.hpp"

using namespace std::chrono_literals;

  MoveAction::MoveAction(std::string node_name, std::chrono::nanoseconds time_)
  : plansys2::ActionExecutorClient(node_name, time_)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.header.stamp = now();

    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    locations_["hall"] = wp;

    wp.pose.position.x = 24.0;
    wp.pose.position.y = -4.6;
    locations_["corridor1"] = wp;
    
    wp.pose.position.x = 24.0;
    wp.pose.position.y = 5.0;
    locations_["corridor2"] = wp;

    wp.pose.position.x = 18.7;
    wp.pose.position.y = -0.7;
    locations_["corridor3"] = wp;

    wp.pose.position.x = 24.3;
    wp.pose.position.y = 0.0;
    locations_["corridor4"] = wp;

    wp.pose.position.x = 34.2;
    wp.pose.position.y = 0.0;
    locations_["corridor5"] = wp;

    wp.pose.position.x = 43.0;
    wp.pose.position.y = -4.7;
    locations_["corridor6"] = wp;

    wp.pose.position.x = 8.36;
    wp.pose.position.y = 0.0;
    locations_["reception"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
  }




  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = locations_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }



 