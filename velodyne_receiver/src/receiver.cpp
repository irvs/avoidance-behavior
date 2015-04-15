#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <iostream>
#include <pcl/conversions.h>

//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
/*
//最小距離確認用
float dddd;
float minmin = 3.0;
*/

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);

  ROS_INFO("size:%d",cloud.points.size());

  for(int i=0; i<10; i++) {
    ROS_INFO("points[%03d]:(%.3f, %.3f, %.3f)", i, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }

  float dddd;
  float minmin = 3.0;
//最小距離確認用
  for(int i=0; i < cloud.points.size(); i++) {
    dddd = sqrt((cloud.points[i].x)*(cloud.points[i].x) + (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].z)*(cloud.points[i].z));
    if(dddd < minmin){
      minmin = dddd;
    }
  }
  ROS_INFO("min_range = %f",minmin);
}

int main(int argc, char **argv) {

  ROS_INFO("Succeed to call pcd receiver");
  ros::init(argc, argv, "pcd_receiver");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("velodyne_points", 10, pointsCallback);

  //ROS_INFO("spin start");
  ros::spin();
  //ROS_INFO("spin end");
  return 0;
}

/* Pointcloud2の型

bool is_bigendian = 0
uint32 point_step : ポイントの長さ 32[byte]
uint32 row_step : 列の長さ[byte]
uint8[] data : サイズはrow_step
bool is_dense = 1
string frame_id
uint32 height = 1
uint32 width

std_msgs/Header header
  uint32 seq
  time stamp

sensor_msgs/PointField[] fields
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
 */

