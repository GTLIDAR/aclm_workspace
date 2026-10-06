
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <visualization_msgs/MarkerArray.h>
#include <ocs2_multi_robot/constraint/Obstacles.h>
#include <std_msgs/Bool.h>
#include <ocs2_core/misc/LoadData.h>

class ObstacleVisualization
{
public:
    std::string frameId_ = "odom"; // Frame name all messages are published in

    ObstacleVisualization(ros::NodeHandle &nodehandle, const std::string taskFile) : nh(nodehandle)
    {
        obstaclesPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 1);
        obstaclesTimer_ = nh.createTimer(ros::Duration(0.1), &ObstacleVisualization::timerCallback, this);

        ocs2::loadData::loadStdVector(taskFile, "obstacles.radius", obstacles_radius, false);

        // Load use_perceptive parameter to switch between terrain and flat cases
        nh.param<bool>("use_perceptive", usePerceptive_, false);
        ROS_INFO_STREAM("[ObstacleVisualization] use_perceptive: " << (usePerceptive_ ? "true (terrain)" : "false (flat)"));
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        visualization_msgs::MarkerArray markerArray;
        ros::Time timeStamp = ros::Time::now();

        // Obstacle box 1
        visualization_msgs::Marker marker;

        marker.header.frame_id = frameId_;
        marker.header.stamp = timeStamp;
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.pose.position.x = 2.0;
        marker.pose.position.y = 6.0;
        marker.pose.position.z = usePerceptive_ ? 1.35 : 0.75;  // 1.35 for terrain, 0.75 for flat

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        markerArray.markers.push_back(marker);

        // Obstacle box 2
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;

        marker.pose.position.x = -2.0;
        marker.pose.position.y = 7.0;
        marker.pose.position.z = usePerceptive_ ? 1.35 : 0.75;  // 1.35 for terrain, 0.75 for flat

        markerArray.markers.push_back(marker);

        // Obstacle wall 1 (left)
        marker.id = 2;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 4.0;
        marker.scale.y = 0.5;
        marker.scale.z = 2.0;

        marker.pose.position.x = -2.5;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = usePerceptive_ ? 1.6 : 1.0;  // 1.6 for terrain, 1.0 for flat

        markerArray.markers.push_back(marker);

        // Obstacle wall 2 (right)
        marker.id = 3;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 3.0;
        marker.scale.y = 0.5;
        marker.scale.z = 2.0;

        marker.pose.position.x = 3.5;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = usePerceptive_ ? 1.6 : 1.0;  // 1.6 for terrain, 1.0 for flat

        markerArray.markers.push_back(marker);

        // Obstacle wall 3 (upper)
        marker.id = 4;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 5.0;
        marker.scale.y = 0.5;
        marker.scale.z = 1.2;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 9.0;
        marker.pose.position.z = usePerceptive_ ? 3.0 : 1.8;  // 3.0 for terrain(slope), 1.8 for flat

        markerArray.markers.push_back(marker);

        // Obstacle wall 1 & 2 (spheres)
        // marker.id = 5;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.action = visualization_msgs::Marker::ADD;
        // double sphere1_radius = 1.3;
        // marker.scale.x = 2.0 * sphere1_radius;
        // marker.scale.y = 2.0 * sphere1_radius;
        // marker.scale.z = 2.0 * sphere1_radius;

        // marker.pose.position.x = -1.2;
        // marker.pose.position.y = 2.0;
        // marker.pose.position.z = usePerceptive_ ? 1.6 : 1.0;  // 1.6 for terrain, 1.0 for flat
        // markerArray.markers.push_back(marker);

        // marker.id = 6;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.action = visualization_msgs::Marker::ADD;
        // double sphere2_radius = 1.3;
        // marker.scale.x = 2.0 * sphere2_radius;
        // marker.scale.y = 2.0 * sphere2_radius;
        // marker.scale.z = 2.0 * sphere2_radius;

        // marker.pose.position.x = 2.7;
        // marker.pose.position.y = 2.0;
        // marker.pose.position.z = usePerceptive_ ? 1.6 : 1.0;  // 1.6 for terrain, 1.0 for flat
        // markerArray.markers.push_back(marker);

        obstaclesPublisher_.publish(markerArray);
    }

private:
    ros::NodeHandle nh;
    ros::Timer obstaclesTimer_;
    ros::Publisher obstaclesPublisher_;
    ocs2::scalar_array_t obstacles_radius;
    bool usePerceptive_ = false;  // Flag to switch between terrain and flat cases
};
