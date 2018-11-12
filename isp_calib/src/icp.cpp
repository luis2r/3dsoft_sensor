#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>



#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


//typedef pcl::PointCloud<pcl::pcl::PointXYZ> PointCloud2;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("points2", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);  
  // msg->header.frame_id = "some_tf_frame";

// Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;

  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;

  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);

  pcl::PointCloud<pcl::PointXYZ> Final;
  // PointCloud::Ptr Final(new PointCloud);

  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<

  icp.getFitnessScore() << std::endl;

  std::cout << icp.getFinalTransformation() << std::endl;



  Final.header.frame_id = "some_tf_frame";




  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    //pcl_conversions::toPCL(ros::Time::now(), Final->header.stamp);
    pub.publish (Final);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}