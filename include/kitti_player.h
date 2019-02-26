#ifndef KITTI_PLAYER_H
#define KITTI_PLAYER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Geometry>

class KittiPlayer
{
public:
    KittiPlayer();
    ~KittiPlayer() {}
    void loop();

private:
    void readGroundTruth();
    bool publishPointCloud(const ros::Time &time, int idx);
    bool publishImage(const ros::Time &time, int idx);

private:
    ros::Publisher ground_truth_pub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher image_pub_;
    nav_msgs::Path ground_truth_path_;

    double publish_freq_;
    std::string map_frame_;
    std::string lidar_frame_;

    int kitti_sequence_;
    std::string kitti_dataset_dir_;
    std::string ground_truth_file_;
    std::string point_cloud_file_;
    std::string image_file_;
    std::string calibration_file_;

    int idx_ = 0;
};

#endif // KITTI_PLAYER_H
