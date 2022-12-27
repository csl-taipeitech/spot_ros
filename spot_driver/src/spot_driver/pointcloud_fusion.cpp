#include <ros/ros.h>

// ros msg
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// pcl lib
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

std::string base_frame_id = "body";

pcl::PointCloud<pcl::PointXYZ>::Ptr points_fl (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_fr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_l (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_r (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_b (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_v (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 points2;

geometry_msgs::TransformStamped transform_fl, transform_fr, transform_l, transform_r, transform_b, transform_v;

bool enable_frontleft, enable_frontright, enable_left, enable_right, enable_back, enable_velodyne;

ros::Publisher points_pub;

void getParam(const ros::NodeHandle & node)
{
  node.param<bool>("enable_frontleft", enable_frontleft, true);
  node.param<bool>("enable_frontright", enable_frontright, true);
  node.param<bool>("enable_left", enable_left, true);
  node.param<bool>("enable_right", enable_right, true);
  node.param<bool>("enable_back", enable_back, true);
  node.param<bool>("enable_velodyne", enable_velodyne, true);
}

void point_fl_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_frontleft) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_fl->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_fl, transform_fl.transform);
}

void point_fr_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_frontright) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_fr->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_fr, transform_fr.transform);
}

void point_l_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_left) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_l->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_l, transform_l.transform);
}

void point_r_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_right) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_r->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_r, transform_r.transform);
}

void point_b_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_back) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_b->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_b, transform_b.transform);
}

void point_v_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  if (!enable_velodyne) return;
  static pcl::PointCloud<pcl::PointXYZ> points_tmp;
  // points_v->points.clear();
  pcl::fromROSMsg(*pt_ptr, points_tmp);
  pcl_ros::transformPointCloud(points_tmp, *points_v, transform_v.transform);
}

void timerCallback(const ros::TimerEvent & event)
{
  static pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);

  points->points.clear();

  *points += *points_fl;
  *points += *points_fr;
  *points += *points_l;
  *points += *points_r;
  *points += *points_b;
  *points += *points_v;
  
  pcl::toROSMsg(*points, points2);
  points2.header.frame_id = base_frame_id;
  points2.header.stamp = ros::Time::now();
  points_pub.publish(points2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_fusion");
  ros::NodeHandle nh, nh_p("~");

  getParam(nh_p);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while (true)
  {
    try
    {
      transform_fl = tfBuffer.lookupTransform(base_frame_id, "frontleft", ros::Time(0));
      transform_fr = tfBuffer.lookupTransform(base_frame_id, "frontright", ros::Time(0));
      transform_l = tfBuffer.lookupTransform(base_frame_id, "left", ros::Time(0));
      transform_r = tfBuffer.lookupTransform(base_frame_id, "right", ros::Time(0));
      transform_b = tfBuffer.lookupTransform(base_frame_id, "back", ros::Time(0));
      transform_v = tfBuffer.lookupTransform(base_frame_id, "velodyne", ros::Time(0));
      break;
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }
  }

  ros::Subscriber frontleft_points_sub  = nh.subscribe("frontleft_points", 1,   point_fl_cb);
  ros::Subscriber frontright_points_sub = nh.subscribe("frontright_points", 1,  point_fr_cb);
  ros::Subscriber left_points_sub       = nh.subscribe("left_points", 1,        point_l_cb);
  ros::Subscriber right_points_sub      = nh.subscribe("right_points", 1,       point_r_cb);
  ros::Subscriber back_points_sub       = nh.subscribe("back_points", 1,        point_b_cb);
  ros::Subscriber velodyne_points_sub   = nh.subscribe("velodyne_points", 1,    point_v_cb);

  points_pub = nh.advertise<sensor_msgs::PointCloud2>("surround_points", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();

  ros::spin();

  return 0;
}