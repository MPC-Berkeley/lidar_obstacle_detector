# lidar_obstacle_detector

3D LiDAR Object Detection &amp; Tracking using Euclidean Clustering &amp; Hungarian algorithm

![demo_1](media/demo_1.gif)

![demo_2](media/demo_2.gif)

## Features
* Segmentation of ground plane and obstacle point clouds
* Customizable Region of Interest (ROI) for obstacle detection
* Customizable region for removing ego vehicle points from the point cloud
* Tracking of obstacles between frames using IOU gauge and Hungarian algorithm

**TODOs**
* 

**Known Issues**
*

## Dependencies
* autoware-msgs
* jsk-recognition-msgs

## Installation
```bash
# clone the repo
cd catkin_ws/src
git clone https://github.com/SS47816/lidar_obstacle_detector.git

# install dependencies & build 
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage

### (Easy) Use this pkg with ROS Bags (`mai_city` dataset as an example here)

**Step 1**: Download the `mai_city` dataset from their [Official Website](https://www.ipb.uni-bonn.de/data/mai-city-dataset/)

**Step 2**: Launch the node using the `mai_city.launch` file
```bash
# this will launch the obstacle_detector node, rviz, and rqt_reconfigure GUI together
roslaunch lidar_obstacle_detector mai_city.launch
```

**Step 3**: Run any of the bags from the dataset
```bash
# go to the folder where the dataset is located
cd mai_city/bags
# play the rosbag
rosbag play 00.bag
```

### Use this pkg with LGSVL Simulator (and my `lgsvl_utils` pkg)

**Step 1**: Change the topic names and params in the `launch/lidar_obstacle_detector.launch` file

**Step 2**: Launch using the launch file
```bash
# launch node
roslaunch lidar_obstacle_detector lidar_obstacle_detector.launch 
```

**Step 3**: In order to help you tune the parameters to suit your own applications better, all the key parameters of the algorithm are controllable in live action using the ros param dynamic reconfigure feature
```bash
# launch 
rosrun rqt_reconfigure rqt_reconfigure 
```

## Contribution
Contributions are welcomed!

## License
MIT License