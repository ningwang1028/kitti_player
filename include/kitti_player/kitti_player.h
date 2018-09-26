#ifndef KITTI_PLAYER_H
#define KITTI_PLAYER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

class KittiPlayer
{
public:
    KittiPlayer();
    ~KittiPlayer() {}
    void loop();

private:
    void readGroundTruth();
    void publishPointCloud();

private:
    ros::Publisher ground_truth_pub_;
    ros::Publisher point_cloud_pub_;
    nav_msgs::Path ground_truth_path_;

    double publish_freq_;
    std::string map_frame_;
    std::string lidar_frame_;
    std::string ground_truth_file_;
    std::string point_cloud_file_;
    int cloud_idx_;
};

#endif // KITTI_PLAYER_H
