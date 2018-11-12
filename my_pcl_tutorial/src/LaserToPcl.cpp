#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
//#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

class LaserToCloud
{
    public:
        //typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        //bool cloud_completed = false;
        LaserToCloud();
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    private:
        ros::NodeHandle nh;
        laser_geometry::LaserProjection projector;
        tf::TransformListener tfListener;
        ros::Subscriber laser_sub;
        ros::Publisher pub_cloud;
        tf::TransformListener listener;

    //pub_cloud = nh.advertise<pcl::PointXYZ> ("laser_points", 1);

};


LaserToCloud::LaserToCloud()
{
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &LaserToCloud::laser_callback, this);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cloud", 1, false);
    //tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserToCloud::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	// if(cloud_completed == false)
	// {

	// }
	// else
	// {

    //sensor_msgs::PointCloud2 cloud;
    //projector.projectLaser(*scan_in, cloud);
    //projector.transformLaserScanToPointCloud("axis", *scan_in, cloud, tfListener);




  if(!listener.waitForTransform( scan_in->header.frame_id, "/axis", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))
  {
     return;

  }

  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("axis",*scan_in, cloud,listener);
  // Do something with cloud.






    pub_cloud.publish(cloud);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laser_to_cloud");
    LaserToCloud lasertocloud;
    ros::spin();

}