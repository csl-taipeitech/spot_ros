#include <ros/ros.h>

// ros msg
#include <sensor_msgs/PointCloud2.h>

// pcl lib
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 points2;
ros::Publisher points_pub;

void point_cb(const sensor_msgs::PointCloud2Ptr & pt_ptr)
{
  static pcl::PassThrough<pcl::PointXYZ> pass;
  
  pcl::fromROSMsg(*pt_ptr, *points);
  if (points->points.size() == 0)
  {
    return;
  }

  pass.setInputCloud(points);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.2, 1.0);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.22, 0.22);
  pass.setNegative(true);
  pass.filter(*points);

  pcl::toROSMsg(*points, points2);
  points_pub.publish(points2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_pass");
  ros::NodeHandle nh;

  ros::Subscriber points_sub = nh.subscribe("velodyne_points", 1, point_cb);

  points_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_filtered", 1);

  ros::spin();

  return 0;
}