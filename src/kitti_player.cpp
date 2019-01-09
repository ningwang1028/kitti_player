#include "kitti_player.h"
#include <fstream>
#include <stdio.h>
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

    usleep(2000000);
}

void KittiPlayer::readGroundTruth()
{
    std::ifstream trajectory_stream(ground_truth_file_);
    if(!trajectory_stream.is_open()) {
        std::cout << "could not open " << ground_truth_file_ << std::endl;
        exit(1);
    }

    Eigen::Matrix4f velodyne_to_camera = Eigen::Matrix4f::Identity();
    velodyne_to_camera << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,-1.198459927713e-02,
                           -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                           9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01;
    Eigen::Matrix4f camera_to_velodyne = Eigen::Isometry3f(velodyne_to_camera).inverse().matrix();

    while(!trajectory_stream.eof()) {

        Eigen::Matrix4f pose(Eigen::Matrix4f::Identity());
        trajectory_stream >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3)
                          >> pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3)
                          >> pose(2, 0) >> pose(2, 1) >> pose(2, 2) >> pose(2, 3);
        pose = camera_to_velodyne * pose * velodyne_to_camera;

        trajectory_stream.get();
        if(trajectory_stream.peek() == '/n') {
            break;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.position.x = pose(0, 3);
        pose_msg.pose.position.y = pose(1, 3);
        pose_msg.pose.position.z = pose(2, 3);

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

bool KittiPlayer::publishPointCloud()
{
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << cloud_idx_ << ".bin";
    std::string file_name = point_cloud_file_ + "/" + ss.str();
    cloud_idx_++;

    std::ifstream input(file_name, std::ios::in | std::ios::binary);

    if(!input.is_open()) {
        return false;
    }

    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (int i = 0; input.good() && !input.eof(); ++i) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }
    input.close();
    std::cout << "Sequence: " << cloud_idx_ <<  " Cloud size: " << cloud.size() << std::endl;

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(cloud, point_cloud_msg);
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.header.frame_id = lidar_frame_;
    point_cloud_pub_.publish(point_cloud_msg);

    return true;
}

void KittiPlayer::loop()
{
    ros::Rate rate(publish_freq_);

    while(ros::ok()) {
        if(!publishPointCloud())
            return;

        rate.sleep();
    }
}
