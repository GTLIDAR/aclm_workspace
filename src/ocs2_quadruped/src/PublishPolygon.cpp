//
// Created by ziyi on 9/28/22.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <Eigen/Dense>

const Eigen::Vector2d SquareSize{10, 10};

visualization_msgs::Marker generateRebarMarkerArrayFromFile(const std::string& setup_path) {
  Json::Value rebar_file;
  std::ifstream rebar_fstream(setup_path);
  rebar_fstream >> rebar_file;
  auto rebar_setup = rebar_file["rebar_setup"];
  if(rebar_setup.getMemberNames().empty()){
    throw std::runtime_error("[Rebar visualizer] The rebar layout is not specified correctly");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rebar_ends;
  for (auto r : rebar_setup["spacing"]) {
    Eigen::Vector3d end1, end2;
    end1 << r[0][0].asDouble(), r[0][1].asDouble(), 0.05;
    end2 << r[1][0].asDouble(), r[1][1].asDouble(), 0.05;
    rebar_ends.push_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(end1, end2));
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "odom";
  line_list.id = 0;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.pose.orientation.w = 1.0;
  line_list.scale.x = 0.01;
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;
  for (int i = 0; i < rebar_ends.size(); i++) {
    auto r = rebar_ends[i];
    geometry_msgs::Point p1, p2;
    p1.x = r.first[0];
    p1.y = r.first[1];
    p1.z = r.first[2];
    p2.x = r.second[0];
    p2.y = r.second[1];
    p2.z = r.second[2];

    line_list.points.push_back(p1);
    line_list.points.push_back(p2); 
  }
  
  // Json::Value ik_setup;
  // std::ifstream ik_fstream(setup_path);
  // ik_fstream >> ik_setup;
  // visualization_msgs::Marker marker_array;

  // auto rebar_setup = ik_setup["rebar_setup"];
  // if(!rebar_setup.getMemberNames().empty()){
  //   ROS_ASSERT(rebar_setup.getMemberNames().size() == 1);
  //   double spacing_x = rebar_setup["spacing"][0].asDouble();
  //   double spacing_y = rebar_setup["spacing"][1].asDouble();

  //   // Determine the vertices of each cube(rebar)
  //   int num_rebar_one_side_x = int(double(SquareSize[0]/spacing_x));
  //   int num_rebar_one_side_y = int(double(SquareSize[1]/spacing_y));
  //   // Assume the tying postion is always [0, 0]
  //   for (int i = -num_rebar_one_side_x; i < num_rebar_one_side_x+1; ++i) {
  //     double x_pos = 0.0 + spacing_x*i;
  //     visualization_msgs::Marker marker;
  //     marker.header.frame_id = "odom";
  //     marker.id = i;
  //     marker.type = visualization_msgs::Marker::CUBE;
  //     marker.pose.position.x = x_pos;
  //     marker.pose.position.y = 0.0;
  //     marker.pose.position.z = 0.05;
  //     marker.pose.orientation.w = 1.0;
  //     marker.pose.orientation.x = 0.0;
  //     marker.pose.orientation.y = 0.0;
  //     marker.pose.orientation.z = 0.0;
  //     marker.scale.x = 0.01;
  //     marker.scale.y = SquareSize[1]*2;
  //     marker.scale.z = 0.005;
  //     marker.color.a = 0.6; // Don't forget to set the alpha!
  //     marker.color.r = 0.0;
  //     marker.color.g = 1.0;
  //     marker.color.b = 0.0;
  //     marker_array.markers.push_back(marker);
  //   }

  //   for (int i = -num_rebar_one_side_y; i < num_rebar_one_side_y+1; ++i) {
  //     double y_pos = 0.0 + spacing_y*i;
  //     visualization_msgs::Marker marker;
  //     marker.header.frame_id = "odom";
  //     marker.id = i+2*num_rebar_one_side_x+1;
  //     marker.type = visualization_msgs::Marker::CUBE;
  //     marker.pose.position.x = 0.0;
  //     marker.pose.position.y = y_pos;
  //     marker.pose.position.z = 0.05;
  //     marker.pose.orientation.w = 1.0;
  //     marker.pose.orientation.x = 0.0;
  //     marker.pose.orientation.y = 0.0;
  //     marker.pose.orientation.z = 0.0;
  //     marker.scale.x = SquareSize[0]*2;
  //     marker.scale.y = 0.01;
  //     marker.scale.z = 0.005;
  //     marker.color.a = 0.6; // Don't forget to set the alpha!
  //     marker.color.r = 0.0;
  //     marker.color.g = 1.0;
  //     marker.color.b = 0.0;
  //     marker_array.markers.push_back(marker);
  //   }

  //   // Publish the tying point
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = "odom";
  //   marker.id = 4*num_rebar_one_side_x+2;
  //   marker.type = visualization_msgs::Marker::SPHERE;
  //   marker.pose.position.x = 0.0;
  //   marker.pose.position.y = 0.0;
  //   marker.pose.position.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.scale.x = 0.04;
  //   marker.scale.y = 0.04;
  //   marker.scale.z = 0.04;
  //   marker.color.a = 1.0; // Don't forget to set the alpha!
  //   marker.color.r = 1.0;
  //   marker.color.g = 0.0;
  //   marker.color.b = 0.0;
  //   marker_array.markers.push_back(marker);
  // }

  return line_list;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "polygon_publisher");
  ros::NodeHandle nh;
  std::string rebar_file;
  nh.getParam("/rebarFile", rebar_file);
  
  // Publish the rebar using cubes
  ros::Publisher rebar_pub = nh.advertise<visualization_msgs::Marker>("polygon_visualization", 1);
  ros::Rate rate(1);
  auto rebar_marker = generateRebarMarkerArrayFromFile(rebar_file);

  while(ros::ok()){
    rebar_pub.publish(rebar_marker);
    rate.sleep();
  }

  return 1;
}