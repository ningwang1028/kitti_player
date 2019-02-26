#include "kitti_player.h"
#include <fstream>
#include <stdio.h>
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

KittiPlayer::KittiPlayer()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("publish_freq", publish_freq_, 2.0);
    private_nh.param("lidar_frame", lidar_frame_, std::string("velodyne"));
    private_nh.param("map_frame", map_frame_, std::string("map"));

    private_nh.param("kitti_sequence", kitti_sequence_, 0);
    private_nh.param("kitti_dataset_dir", kitti_dataset_dir_, std::string("/home/ning/data/kitti"));

    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << kitti_sequence_;
    ground_truth_file_ = kitti_dataset_dir_ + "/dataset/poses/" + ss.str() + ".txt";
    point_cloud_file_ = kitti_dataset_dir_ + "/" + ss.str() + "/velodyne";
    image_file_ = kitti_dataset_dir_ + "/" + ss.str() + "/image_0";
    calibration_file_ = kitti_dataset_dir_ + "/calibration/sequences/" + ss.str() + "/calib.txt";

    ground_truth_pub_ = nh.advertise<nav_msgs::Path>("ground_truth", 1, true);
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10, true);
    image_pub_ = nh.advertise<sensor_msgs::Image>("image_raw", 10, true);

    readGroundTruth();

    usleep(2000000);
}

void KittiPlayer::readGroundTruth()
{
    std::ifstream trajectory_stream(ground_truth_file_);
    if(!trajectory_stream.is_open()) {
        std::cout << "could not open " << ground_truth_file_ << std::endl;
        return;
    }

    std::ifstream calibration_stream(calibration_file_);
    if(!calibration_stream) {
        std::cout << "could not open " << calibration_file_ << std::endl;
    }
    std::string line;
    while(std::getline(calibration_stream, line)) {
        if(line.substr(0, 3).compare("Tr:") == 0) {
            break;
        }
    }
    if(line.substr(0, 3).compare("Tr:") != 0) {
        std::cout << "error calibration file!" << std::endl;
        return;
    }

    std::stringstream ss(line.substr(3));
    Eigen::Matrix4f velodyne_to_camera = Eigen::Matrix4f::Identity();
    ss >> velodyne_to_camera(0, 0) >> velodyne_to_camera(0, 1) >> velodyne_to_camera(0, 2) >> velodyne_to_camera(0, 3)
       >> velodyne_to_camera(1, 0) >> velodyne_to_camera(1, 1) >> velodyne_to_camera(1, 2) >> velodyne_to_camera(1, 3)
       >> velodyne_to_camera(2, 0) >> velodyne_to_camera(2, 1) >> velodyne_to_camera(2, 2) >> velodyne_to_camera(2, 3);
    std::cout << velodyne_to_camera << std::endl;
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

bool KittiPlayer::publishImage(const ros::Time& time, int idx)
{
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << idx << ".png";
    std::string file_name = image_file_ + "/" + ss.str();

    cv::Mat image = cv::imread(file_name, 0);
    if(image.empty()) {
        return false;
    }

    std_msgs::Header header;
    header.stamp = time;
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
    image_pub_.publish(image_msg);

    return true;
}

bool KittiPlayer::publishPointCloud(const ros::Time& time, int idx)
{
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << idx << ".bin";
    std::string file_name = point_cloud_file_ + "/" + ss.str();

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

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(cloud, point_cloud_msg);
    point_cloud_msg.header.stamp = time;
    point_cloud_msg.header.frame_id = lidar_frame_;
    point_cloud_pub_.publish(point_cloud_msg);

    return true;
}

void KittiPlayer::loop()
{
    ros::Rate rate(publish_freq_);

    while(ros::ok()) {
        ros::Time t = ros::Time::now();
        std::cout << "Sequence: " << idx_ << std::endl;
        if(!publishPointCloud(t, idx_))
            return;

//        if(!publishImage(t, idx_))
//            return;

        idx_++;
        rate.sleep();
    }
}
