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

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "upf_msgs/srv/pddl_plan_one_shot.hpp"
#include "upf_msgs/msg/pddl_plan_request.hpp"
#include "upf_msgs/msg/plan_generation_result.hpp"

#include "plansys2_upf_plan_solver/upf_plan_solver.hpp"

namespace plansys2
{

using namespace std::chrono_literals;

UPFPlanSolver::UPFPlanSolver()
{
}

plansys2_msgs::msg::PlanItem
UPFPlanSolver::convert(const upf_msgs::msg::ActionInstance & action)
{
  plansys2_msgs::msg::PlanItem ret;

  float start_time = action.start_time.numerator / action.start_time.denominator;
  float end_time = action.end_time.numerator / action.end_time.denominator;

  ret.time = action.start_time.numerator / action.start_time.denominator;
  ret.duration = end_time - start_time;

  ret.action = "(" + action.action_name;
  for (const auto & parameter : action.parameters) {
    if (!parameter.symbol_atom.empty()) {
      ret.action = ret.action + " " + parameter.symbol_atom[0];
    } else if (!parameter.int_atom.empty()) {
      ret.action = ret.action + " " + std::to_string(parameter.int_atom[0]);
    } else if (!parameter.real_atom.empty()) {
      float float_value = parameter.real_atom[0].numerator / parameter.real_atom[0].denominator;
      ret.action = ret.action + " " + std::to_string(float_value);
    } else if (!parameter.boolean_atom.empty()) {
      ret.action = ret.action + " " + (parameter.boolean_atom[0]? "true" : "false");
    }
  }
  ret.action = ret.action + ")";

  return ret;
}

plansys2_msgs::msg::Plan
UPFPlanSolver::convert(const upf_msgs::msg::Plan & plan)
{
  plansys2_msgs::msg::Plan ret;

  for (const auto & action : plan.actions) {
    ret.items.push_back(convert(action));
  }

  return ret;
}

void UPFPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & lc_node,
  const std::string & plugin_name)
{
  parameter_name_ = plugin_name + ".arguments";
  lc_node_ = lc_node;
  lc_node_->declare_parameter<std::string>(parameter_name_, "");
  service_node_ = rclcpp::Node::make_shared(plugin_name + "_node");
}

std::optional<plansys2_msgs::msg::Plan>
UPFPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return {};
  }

  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  auto client = service_node_->create_client<upf_msgs::srv::PDDLPlanOneShot>(
    "upf4ros2/srv/planOneShotPDDL");


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        service_node_->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return {};
    }
    RCLCPP_WARN(service_node_->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<upf_msgs::srv::PDDLPlanOneShot::Request>();
  request->plan_request.mode = upf_msgs::msg::PDDLPlanRequest::FILE;
  request->plan_request.domain = "/tmp/" + node_namespace + "/domain.pddl";
  request->plan_request.problem = "/tmp/" + node_namespace + "/problem.pddl";

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(service_node_, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      service_node_->get_logger(),
      "Failed to call service upf4ros2/srv/planOneShotPDDL");
    return {};
  }

  auto res = *result.get();
  upf_msgs::msg::PlanGenerationResult & plan_result = res.plan_result;

  if (!res.success || 
      (plan_result.status != upf_msgs::msg::PlanGenerationResult::SOLVED_SATISFICING && 
      plan_result.status != upf_msgs::msg::PlanGenerationResult::SOLVED_OPTIMALLY))
  {
    RCLCPP_ERROR(
      service_node_->get_logger(),
      "Failed resolving plan %s", res.message.c_str());
    return {};
  }

  plansys2_msgs::msg::Plan ret = convert(plan_result.plan);

  return ret;
}

bool
UPFPlanSolver::is_valid_domain(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return false;
  }

  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        temp_dir /= p;
      }
    }
    std::filesystem::create_directories(temp_dir);
  }

  std::ofstream domain_out(temp_dir.string() + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out(temp_dir.string() + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  int status = system(
    ("ros2 run popf popf " + temp_dir.string() + "/check_domain.pddl " + temp_dir.string() +
    "/check_problem.pddl > " + temp_dir.string() + "/check.out").c_str());

  if (status == -1) {
    return false;
  }

  std::string line;
  std::ifstream plan_file(temp_dir.string() + "/check.out");
  bool solution = false;

  if (plan_file && plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      }
    }
    plan_file.close();
  }

  return solution;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::UPFPlanSolver, plansys2::PlanSolverBase);
