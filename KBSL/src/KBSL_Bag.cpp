#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <sensor_msgs/PointCloud.h>

#include "KBSL/TransformPointSrv.h"
#include "KBSL/FitMinimalPlaneSrv.h"
#include "KBSL/FindInliersSrv.h"
#include "KBSL/FitBestPlaneSrv.h"
#include "KBSL/PlaneParametersMsg.h"

using namespace std;

// Publisher for point clouds.
ros::Publisher kinect_publisher;

void PointCloudCallback(const sensor_msgs::PointCloud& point_cloud_msg) {
  kinect_publisher.publish(point_cloud_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "KBSLBag");
  ros::NodeHandle n;

  kinect_publisher = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloud", 1);
  ros::Subscriber point_cloud_subscriber = n.subscribe("/COMPSCI403/PointCloudIn", 1, PointCloudCallback);

  ros::spin();

  return 0;
}
