﻿#ifndef _VELODYNE_BAR_H_INCLUDED
#define _VELODYNE_BAR_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/LazyCaller>
#include <cnoid/SceneView>
#include <Grasp/exportdef.h>
#include <Grasp/PlanBase.h>
#include <PRM/TrajectoryPlanner.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <Eigen/Dense>

#include <sstream>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>



#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tms_msg_rc/smartpal_control.h>

//#define PI 3.1415923

//#include "./extplugin/graspPlugin/Grasp/PlanBase.h"

using namespace std;
using namespace cnoid;

namespace grasp {
class EXCADE_API VelodyneBar : public cnoid::ToolBar, public boost::signals::trackable {
 public:
  VelodyneBar();
  static VelodyneBar* instance();
  virtual ~VelodyneBar();

  int argc;
  char **argv;

  ros::Subscriber subscribe_pcd;
  static bool isRosInit;
  pcl::PointCloud<pcl::PointXYZ> pointCloudData;

  ros::ServiceClient control_client;
  tms_msg_rc::smartpal_control sp_control_srv;

  boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
    return sigBodyItemSelectionChanged_;
  }

 private:
  static int count;

  MessageView& mes;
  std::ostream& os;

  cnoid::BodyItemPtr currentBodyItem_;
  cnoid::ItemList<cnoid::BodyItem> selectedBodyItems_;
  cnoid::ItemList<cnoid::BodyItem> targetBodyItems;

  boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;

  void onItemSelectionChanged(const cnoid::ItemList<cnoid::BodyItem>& bodyItems);
  void onTestButtonClicked();
  void onPCDButtonClicked();
  void onSTARTButtonClicked();
  void onCollisionButtonClicked();
  void viewCollidingPoint(double x, double y, double z);
  void viewCollidingPoints(vector<Vector3>& p);
  void onPlusButtonClicked();
  void onMinusButtonClicked();
  void addX();
  void subX();
  void addY();
  void subY();
  void addZ();
  void subZ();
  void calcArmPosition();
  void moveArm(double x, double y, double z);
  void moveArm2(double x, double y, double z);
  void moveElbow(double x, double y, double z);
  void moveArm3(double x, double y, double z);
  void moveArm4(double x, double y, double z);
  void checkModelCollision();
  void changeTheta();
  void plusTheta();
  void minusTheta();
  void onResetButtonClicked();
  void onResetButtonClicked2();
  void onRemoveButtonClicked();
  void viewPosition();
  void setPose();
  void setPoseSimulation();
  void setPoseButtonClicked();
  void getPose();
  void abort();
  void traceTrajectory();
  void traceTrajectorySimulation();
  void evade();
  void evade2();
  void grip();
  void forEvadeAndGrip();
  void rosOn();
  void receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg);



};
}

#endif
