#include "kitti_player.h"
#include <fstream>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

KittiPlayer::KittiPlayer() : cloud_idx_(0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("publish_freq", publish_freq_, 2.0);
    private_nh.param("lidar_frame", lidar_frame_, std::string("velodyne"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("ground_truth_file", ground_truth_file_, std::string("/home/kitti/dataset/poses/00.txt"));
    private_nh.param("point_cloud_file", point_cloud_file_, std::string("/home/kitti/00/velodyne"));

    ground_truth_pub_ = nh.advertise<nav_msgs::Path>("ground_truth", 1, true);
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 100, true);

    readGroundTruth();
}

void KittiPlayer::readGroundTruth()
{
    std::ifstream trajectory_stream(ground_truth_file_);
    if(!trajectory_stream.is_open()) {
        std::cout << "could not open " << ground_truth_file_ << std::endl;
        exit(1);
    }

    Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
    while(!trajectory_stream.eof()) {
        trajectory_stream >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3)
                          >> pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3)
                          >> pose(2, 0) >> pose(2, 1) >> pose(2, 2) >> pose(2, 3);

        trajectory_stream.get();
        if(trajectory_stream.peek() == '/n') {
            break;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.position.x = pose(2, 3);
        pose_msg.pose.position.y = -pose(0, 3);
        pose_msg.pose.position.z = -pose(1, 3);

        tf::Matrix3x3 R;
        double roll, pitch, yaw;
        R.setValue(pose(0, 0), pose(0, 1), pose(0, 2),
                   pose(1, 0), pose(1, 1), pose(1, 2),
                   pose(2, 0), pose(2, 1), pose(2, 2));
        R.getRPY(roll, pitch, yaw);

        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        ground_truth_path_.poses.push_back(pose_msg);

        ground_truth_path_.header.stamp = ros::Time::now();
        ground_truth_path_.header.frame_id = map_frame_;
        ground_truth_pub_.publish(ground_truth_path_);
    }
}

void KittiPlayer::publishPointCloud()
{
    std::cout << "Sequence: " << cloud_idx_;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << cloud_idx_ << ".bin";
    std::string file_name = point_cloud_file_ + "/" + ss.str();
    cloud_idx_++;

    std::fstream input(file_name, std::ios::in | std::ios::binary);
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (int i = 0; input.good() && !input.eof(); ++i) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }
    input.close();
    std::cout << " Cloud size: " << cloud.size() << std::endl;

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(cloud, point_cloud_msg);
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.header.frame_id = lidar_frame_;
    point_cloud_pub_.publish(point_cloud_msg);
}

void KittiPlayer::loop()
{
    ros::Rate rate(publish_freq_);

    while(ros::ok()) {
        publishPointCloud();
        rate.sleep();
    }
}
