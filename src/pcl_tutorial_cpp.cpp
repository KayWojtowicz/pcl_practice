#include "ros/rate.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
std::string camera;
std::string output;
float z_min_filter_limit;
float z_max_filter_limit;
float distance_threshold;
std::string filter_field_name;
float point_color_threshold;
float region_color_threshold;
float min_cluster_size;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName (filter_field_name);
  pass.setFilterLimits (z_min_filter_limit, z_max_filter_limit);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (raw_cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distance_threshold);
  reg.setPointColorThreshold (point_color_threshold);
  reg.setRegionColorThreshold (region_color_threshold);
  reg.setMinClusterSize (min_cluster_size);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  transformed_cloud = reg.getColoredCloud();

  sensor_msgs::PointCloud2 output_msg;

  pcl::toROSMsg(*transformed_cloud, output_msg);
  output_msg.header.frame_id = cloud_msg->header.frame_id;
  output_msg.header.stamp = cloud_msg->header.stamp;
  
  pub.publish(output_msg);
  //how to edit the size of the shown pixels
  /*
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (raw_cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*transformed_cloud);
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_cloud, output_msg);
  */

  pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_tutorial_cloud");
  ros::NodeHandle nh("~");

  nh.param<std::string>("camera_param", camera, "/camera/depth/points");
  nh.param<std::string>("output_param", output, "output");
  

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(camera, 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>(output, 1);
  
  ros::Rate rate(10);
  while (ros::ok())
  {
    nh.param<float>("z_min_filter_limit", z_min_filter_limit, 0.0);
    nh.param<float>("z_max_filter_limit", z_max_filter_limit, 1.0);
    nh.param<float>("distance_threshold", distance_threshold, 10);
    nh.param<std::string>("filter_field_name", filter_field_name, "z");
    nh.param<float>("point_color_threshold", point_color_threshold, 6);
    nh.param<float>("region_color_threshold", region_color_threshold, 5);
    nh.param<float>("min_cluster_size", min_cluster_size, 600);
    
    ros::spinOnce();
    rate.sleep();

  }
}