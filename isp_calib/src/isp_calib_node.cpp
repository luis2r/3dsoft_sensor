#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
sensor_msgs::PointCloud2 output;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  //sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

}

// void cloud2_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   // Container for original & filtered data
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   // Convert to PCL data type
//   pcl_conversions::toPCL(*cloud_msg, *cloud);

//   // Perform the actual filtering
//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloudPtr);
//   sor.setLeafSize (0.1, 0.1, 0.1);
//   sor.filter (cloud_filtered);

//   // Convert to ROS data type
//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered, output);

// }

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("cloud1", 1, cloud_cb);

    // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("cloud2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("icp_output", 1);

  // Publish the data
  pub.publish (output);
  // Spin
  ros::spin ();
}