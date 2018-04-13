# VI_ROS
**VI_ROS package**

This package is a Vegetation Indices generator tool using a Kinect sensor. It can be used to generate VI (Vegetation Indices) images and 3D point clouds of the environment for monitoring vegetation purposes. At the moment only NDVI VI is supported.

**Authors** David Calero <<david.calero@cttc.es>>, Enric Fernandez <<enric.fernandez@cttc.es>>, Centre Tecnològic de Telecomunicacions de Catalunya [CTTC](http://www.cttc.es/) 

## Prerequisites
This project was build with ROS Kinetic but should work on older ROS versions as well. In addition to that, this project depends on the ROS packages [iai_kinect2](https://github.com/code-iai/iai_kinect2), [OpenCV_3.X](https://opencv.org/)  and [pcl_ros](http://wiki.ros.org/pcl_ros). 

## Installation
```
$ cd ~/catkin_ws
$ git https://github.com/CTTCGeoLab/VI_ROS.git src/ndvi
$ catkin_make --pkg ndvi
```

## Configuration
Configuration parameters can be found at the ndvi_kinect.launch file.
```
save_path:=<string>
    default: /home/user/catkin_ws/src/ndvi/data
    info:    set the path folder for the generated output data
save_rgb:=<bool>
    default:  true
    info:    enable saving the input RGB images
save_ir:=<bool>
    default:  true
    info:    enable saving the input IR images
save_depth:=<bool>
    default:  true
    info:    enable saving the input depth images
save_ndvi:=<bool>
    default:  true
    info:    enable saving the output NDVI images
visualize:=<bool>
    default:  true
    info:    enable to visualize the images during execution
publish_ndvi:=<bool>
    default: true
    info:    publish the NDVI image in order to generate NDVI pointclouds
topic_pcl_rgbd:=<string>
    default: /kinect2/sd/points
    info:    set the input topic name of the RGB point cloud generated to save
topic_pcl_ndvi:=<string>
    default: /ndvi/sd/points
    info:    set the topic name of the NDVI point cloud generated to save
topic_pcl_ndvifil1:=<string>
    default: /ndvi/sd/points
    info:    set the topic name of the NDVI point cloud first filtered generated to save
topic_pcl_ndvifil2:=<string>
    default: /ndvi/sd/points
    info:    set the topic name of the NDVI point cloud second filtered generated to save  
```

## Execution
```
$ roslaunch ndvi ndvi_kinect.launch
```

## Screenshots
1) Inputs:
- RGB image

![RGB image](https://raw.githubusercontent.com/CTTCGeoLab/VI_ROS/master/res/BGR_HD_1519205832316048560.png)

- IR image

![IR image](https://raw.githubusercontent.com/CTTCGeoLab/VI_ROS/master/res/IR_151920575742319333.png)

2) Outputs:
- NDVI image

![NDVI image](https://raw.githubusercontent.com/CTTCGeoLab/VI_ROS/master/res/NDVI_151920575742319333.png)

- NDVI Point Cloud

![NDVI PointCloud](https://raw.githubusercontent.com/CTTCGeoLab/VI_ROS/master/res/NDVIPointCloud.png)

- NDVI Point Cloud Filtered

![NDVI PointCloud filtered](https://github.com/CTTCGeoLab/VI_ROS/blob/master/res/NDVIPointCloudFiltered.png)

