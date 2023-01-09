// Copyright 2022 Intelligent Robotics Lab
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

#ifndef PLANSYS2_UPF_PLAN_SOLVER__UPF_PLAN_SOLVER_HPP_
#define PLANSYS2_UPF_PLAN_SOLVER__UPF_PLAN_SOLVER_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "upf_msgs/action/pddl_plan_one_shot.hpp"

namespace plansys2
{

class UPFPlanSolver : public PlanSolverBase
{
private:
  using PDDLPlanOneShot = upf_msgs::action::PDDLPlanOneShot;

  std::string parameter_name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  rclcpp::Node::SharedPtr service_node_;

  plansys2_msgs::msg::Plan convert(const upf_msgs::msg::Plan & plan);
  plansys2_msgs::msg::PlanItem convert(const upf_msgs::msg::ActionInstance & action);

public:
  UPFPlanSolver();

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &);

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  bool is_valid_domain(
    const std::string & domain,
    const std::string & node_namespace = "");
};

}  // namespace plansys2

#endif  // PLANSYS2_UPF_PLAN_SOLVER__UPF_PLAN_SOLVER_HPP_
