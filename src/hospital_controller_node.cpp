// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(GOAL_0)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    //instances
    problem_expert_->addInstance(plansys2::Instance{"brobot", "robot"});
    
    problem_expert_->addInstance(plansys2::Instance{"visit1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"str1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"str2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"str3", "room"});
    problem_expert_->addInstance(plansys2::Instance{"str4", "room"});
    problem_expert_->addInstance(plansys2::Instance{"str5", "room"});
    
    problem_expert_->addInstance(plansys2::Instance{"corridor1", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corridor2", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corridor3", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corridor4", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corridor5", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corridor6", "corridor"});
    
    problem_expert_->addInstance(plansys2::Instance{"reception", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"hall", "corridor"});
    
    //hall
    
    
    problem_expert_->addPredicate(plansys2::Predicate("(connected hall corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 hall)"));
    //*
    problem_expert_->addPredicate(plansys2::Predicate("(connected hall corridor2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 hall)"));
    //corridor1
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 reception)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected reception corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor3 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 corridor1)"));
    
    //corridor2
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 reception)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected reception corridor2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor3 corridor2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 corridor2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 corridor2)"));
    //*/
    
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at brobot hall)"));
  
  }

  void step()
  {
    switch (state_) {
      case GOAL_0:
        {
          // Set the goal for next state TO DO 
          problem_expert_->setGoal(plansys2::Goal("(and(robot_at brobot corridor1))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = GOAL_1;
          }
        }
        break;
      case GOAL_1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Set the goal for next state TO DO 
              problem_expert_->setGoal(plansys2::Goal("(and(robot_at brobot corridor2))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = GOAL_2;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case GOAL_2:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Set the goal for next state TO DO 
              problem_expert_->setGoal(plansys2::Goal("(and(robot_at brobot corridor3))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = GOAL_3;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      case GOAL_3:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Set the goal for next state TO DO
              problem_expert_->setGoal(plansys2::Goal("(and(robot_at brobot corridor4))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = FINISHED;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      
      case FINISHED:
        std::cout << "CONTROLLER: PLAN FINISHED" << std::endl;
        return;
        break;

      default:
        break;
    }
  }

private:
  typedef enum {GOAL_0, GOAL_1, GOAL_2, GOAL_3, FINISHED} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
