#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <tf/transform_listener.h>
//#include "pcl_ros/transforms.h"
//#include <geometry_msgs/Pose.h>
//#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_cloud;
bool cloud_completed = false;
laser_geometry::LaserProjection projector;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(cloud_completed == false)
	{

	}
	else
	{

	}
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);

    //pcl::PointCloud<pcl::PointXYZ> cloudpcl;
    //PointCloud::Ptr cloudpcl (new PointCloud);
    //msg->header.frame_id = "some_tf_frame";
    //cloudpcl->header.frame_id = "some_tf_frame";

    //pcl::fromROSMsg (cloud, *cloudpcl);

    //cloudpcl.header.frame_id = "cloud_frame";
    //cloudpcl.header.stamp = scan_in->header.stamp;
    PointCloud::Ptr cloudpcl (new PointCloud);
    cloudpcl->header.frame_id = "some_tf_frame";
    cloudpcl->height = cloudpcl->width = 1;
    cloudpcl->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

    pcl_conversions::toPCL(ros::Time::now(), cloudpcl->header.stamp);
    //pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    //pub.publish (msg);
    pub_cloud.publish(cloudpcl);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laser_to_cloud");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_callback);

    
    pub_cloud = nh.advertise<PointCloud> ("laser_points", 1);
    //pub_cloud = nh.advertise<pcl::PointXYZ> ("laser_points", 1);

    ros::spin();

}