# kitti_player
ROS package for play kitti dataset into ROS.
## Details
Only play ground truth poses and velodyne data.
#### Message type: 
  - nav_msgs/Path 
  - sensor_msgs/PointCloud2
#### Topics: 
  - /ground_truth 
  - /velodyne_points
## Build
  ```shell 
  cd ${catkin_workspace}/src
  git clone https://github.com/ningwang1028/kitti_player.git
  cd ..
  catkin_make
  ```
## Run
  - change parameters in launch/kitti_player.launch, including your dataset location and publish freqency.  
  - run  
  ```shell  
  roslaunch kitti_player kitti_player.launch
  ```

