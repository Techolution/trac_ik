/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <iostream>
#include <kdl/frames_io.hpp>  // Include for printing KDL frames

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trac_ik/trac_ik.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // Include for Float64MultiArray message
#include "trac_ik_examples/srv/ik_service.hpp"  // Include the custom service message

class IKTestsNode : public rclcpp::Node
{
public:
  IKTestsNode()
    : Node("ik_tests")
  {
    this->declare_parameter<int>("num_samples", 1000);
    this->declare_parameter<double>("timeout", 0.005);
    this->declare_parameters<std::string>(
      std::string(),       // parameters are not namespaced
      std::map<std::string, std::string>{
      {"chain_start", std::string()},
      {"chain_end", std::string()},
      {"robot_description", std::string()},
    });

    this->get_parameter("num_samples", num_samples_);
    this->get_parameter("timeout", timeout_);
    this->get_parameter("chain_start", chain_start_);
    this->get_parameter("chain_end", chain_end_);
    this->get_parameter("robot_description", urdf_xml_);

    if (chain_start_.empty() || chain_end_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Missing chain info in launch file");
      exit(-1);
    }

    if (num_samples_ < 1) {
      num_samples_ = 1;
    }

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("trac_ik_joint_angles", 10);
    service_ = this->create_service<trac_ik_examples::srv::IkService>(
      "ik_service",
      std::bind(&IKTestsNode::handle_ik_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_ik_request(const std::shared_ptr<trac_ik_examples::srv::IkService::Request> request,
                         std::shared_ptr<trac_ik_examples::srv::IkService::Response> response)
  {
    target_pose_.p = KDL::Vector(request->pos_x, request->pos_y, request->pos_z);
    target_pose_.M = KDL::Rotation::Quaternion(request->orient_x, request->orient_y, request->orient_z, request->orient_w);

    TRAC_IK::TRAC_IK tracik_solver(chain_start_, chain_end_, urdf_xml_, timeout_, 1e-5);

    KDL::Chain chain;
    KDL::JntArray ll, ul;

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        RCLCPP_ERROR(this->get_logger(), "There was no valid KDL chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll, ul);

    if (!valid) {
        RCLCPP_ERROR(this->get_logger(), "There were no valid KDL joint limits found");
        return;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv vik_solver(chain);
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, 1e-5);

    KDL::JntArray nominal(chain.getNrOfJoints());
    for (uint j = 0; j < nominal.data.size(); j++) {
        nominal(j) = (ll(j) + ul(j)) / 2.0;
    }

    KDL::JntArray result;
    int rc = tracik_solver.CartToJnt(nominal, target_pose_, result);

    if (rc >= 0) {
        for (unsigned int j = 0; j < result.data.size(); ++j) {
            response->joint_angles.push_back(result(j) * 180.0 / M_PI);  // Convert radians to degrees
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to solve IK");
    }
  }

  int num_samples_;
  double timeout_;
  std::string chain_start_;
  std::string chain_end_;
  std::string urdf_xml_;
  KDL::Frame target_pose_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Service<trac_ik_examples::srv::IkService>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKTestsNode>());
  rclcpp::shutdown();
  return 0;
}
