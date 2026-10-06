//
// Created by ziyi on 9/20/22.
//
#include <ros/ros.h>
#include <Eigen/Dense>
#include <stdio.h>
#include "ocs2_quadruped/FullState.h"
#include "ocs2_quadruped/Output.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

class Handler {
public:
  Handler(){
    full_state_sub = nh_.subscribe("/legged_robot/fullState", 5, &Handler::callbackFullState, this);
    torque_pub = nh_.advertise<ocs2_quadruped::Output>("/legged_robot/estimated_torque", 10);
    force_v.resize(4);
    joint_pos_v.resize(4);
    J_v.resize(4);
    torque_.resize(12);
  }

  void RunTorqueCompute(){
    ros::Rate loop(100);
    while(ros::ok()){
      computeTorque();
      ocs2_quadruped::Output computed_torque;
      for(int i=0; i<12; i++){
        computed_torque.torque[i] = torque_[i];
      }
      torque_pub.publish(computed_torque);
      ros::spinOnce();
      loop.sleep();
    }
  }
  /**
   * Subscribe to the feet forces and store them
   */
  void callbackFullState(const ocs2_quadruped::FullStateConstPtr& msg){
    pos[0] = msg->position[0];
    pos[1] = msg->position[1];
    pos[2] = msg->position[2];

    Eigen::Vector3d zyx_euler;
    zyx_euler << msg->orientation[0], msg->orientation[1], msg->orientation[2];
    w_R_b = ocs2::getRotationMatrixFromZyxEulerAngles(zyx_euler);

    for(int foot=0; foot<4; foot++){
      force_v.at(foot) << msg->footforce[foot*3], msg->footforce[foot*3+1], msg->footforce[foot*3+2];
      joint_pos_v.at(foot) << msg->q[foot*3], msg->q[foot*3+1], msg->q[foot*3+2];
    }

  }

  /**
   * Get the joint torque using the analytical Jacobian
   */
  void computeTorque(){
    for(int foot=0; foot<4; foot++){
      computeLegJacobian(joint_pos_v.at(foot), &J_v.at(foot), foot);
      torque_.middleRows(foot*3, 3) = J_v.at(foot).transpose()*w_R_b.transpose()*force_v.at(foot);
    }
  }

  /**
   * Get the leg jacobian in the local leg coordinate frame, which needs to be transformed to world frame later
   */
  void computeLegJacobian(Eigen::Vector3d& q, Eigen::Matrix3d* J, int leg){
    // the hip and knee joints for A1/GO1 is flipped compared with mini cheetah
    double s1 = std::sin(q(0));
    double s2 = std::sin(-q(1));
    double s3 = std::sin(-q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(-q(1));
    double c3 = std::cos(-q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    if (J) {
      J->operator()(0, 0) = 0;
      J->operator()(0, 1) = -l3 * c23 - l2 * c2;
      J->operator()(0, 2) = -l3 * c23;
      J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSigns[leg] * s1;
      J->operator()(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
      J->operator()(1, 2) = l3 * s1 * s23;
      J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSigns[leg] * c1;
      J->operator()(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
      J->operator()(2, 2) = -l3 * c1 * s23;
    }

  }

private:
  ros::NodeHandle nh_;
  ros::Publisher torque_pub;
  ros::Subscriber full_state_sub;

  Eigen::VectorXd torque_;
  std::vector<Eigen::Matrix3d> J_v;
  std::vector<Eigen::Vector3d> force_v;
  std::vector<Eigen::Vector3d> joint_pos_v;
  Eigen::Matrix3d w_R_b;
  Eigen::Vector3d pos;

  double l1 = 0.0838; // quad._abadLinkLength;
  double l2 = 0.2; // quad._hipLinkLength;
  double l3 = 0.2; // quad._kneeLinkLength;
  double l4 = 0.0; // kneeLinkY_offset;
  double sideSigns[4] = {1, -1, 1, -1};
};




int main (int argc, char** argv){
  ros::init(argc,argv,"compute_torque_node");
  Handler handler;
  handler.RunTorqueCompute();
  return 1;
}