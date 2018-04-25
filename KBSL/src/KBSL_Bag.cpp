/*
Use this to generate a bag file for the Kinect Sensor

Steps:

1. Make sure the Kinect is outputting data to "/COMPSCI403/PointCloudIn"
2. Run this file "rosrun KBSL KBSLBag"
3. Open new terminal and cd into KBSL/bagfiles directory
4. run "rosbag record -a"
 -- might be better to run "rosbag record /COMPSCI403/PointCloud"
5. when done ctrl-c to end the bag file program
*/

#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "KBSL/TransformPointSrv.h"
#include "KBSL/FitMinimalPlaneSrv.h"
#include "KBSL/FindInliersSrv.h"
#include "KBSL/FitBestPlaneSrv.h"
#include "KBSL/PlaneParametersMsg.h"



using namespace std;

// Publisher for point clouds.
ros::Publisher kinect_publisher;

void PointCloudCallback(const sensor_msgs::PointCloud2& point_cloud_msg) {
  kinect_publisher.publish(point_cloud_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "KBSLBag");
  ros::NodeHandle n;

  kinect_publisher = n.advertise<sensor_msgs::PointCloud2>("/COMPSCI403/PointCloud", 1);
  ros::Subscriber point_cloud_subscriber = n.subscribe("/camera/depth/points", 1, PointCloudCallback);

  ros::spin();

  return 0;
}
