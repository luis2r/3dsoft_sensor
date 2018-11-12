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
        void laser1_callback(const sensor_msgs::LaserScan::ConstPtr& scan1_in);
        void laser2_callback(const sensor_msgs::LaserScan::ConstPtr& scan2_in);

    private:
        ros::NodeHandle nh;
        laser_geometry::LaserProjection projector;
        tf::TransformListener tfListener;

        ros::Subscriber laser1_sub;
        ros::Subscriber laser2_sub;

        ros::Publisher pub_cloud1;
        ros::Publisher pub_cloud2;
        tf::TransformListener listener;

    //pub_cloud = nh.advertise<pcl::PointXYZ> ("laser_points", 1);

};


LaserToCloud::LaserToCloud()
{
    laser1_sub = nh.subscribe<sensor_msgs::LaserScan>("scan1", 1, &LaserToCloud::laser1_callback, this);
    laser2_sub = nh.subscribe<sensor_msgs::LaserScan>("scan2", 1, &LaserToCloud::laser2_callback, this);
    pub_cloud1 = nh.advertise<sensor_msgs::PointCloud2> ("cloud1", 1, false);
    pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2> ("cloud2", 1, false);

    //tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserToCloud::laser1_callback(const sensor_msgs::LaserScan::ConstPtr& scan1_in)
{
	// if(cloud_completed == false)
	// {

	// }
	// else
	// {

    //sensor_msgs::PointCloud2 cloud;
    //projector.projectLaser(*scan_in, cloud);
    //projector.transformLaserScanToPointCloud("axis", *scan_in, cloud, tfListener);




  if(!listener.waitForTransform( scan1_in->header.frame_id, "/axis", scan1_in->header.stamp + ros::Duration().fromSec(scan1_in->ranges.size()*scan1_in->time_increment), ros::Duration(1.0)))
  {
     return;

  }

  sensor_msgs::PointCloud2 cloud1;

  projector.transformLaserScanToPointCloud("axis",*scan1_in, cloud1,listener);

  // Do something with cloud.






    pub_cloud1.publish(cloud1);

}
void LaserToCloud::laser2_callback(const sensor_msgs::LaserScan::ConstPtr& scan2_in)
{
	// if(cloud_completed == false)
	// {

	// }
	// else
	// {

    //sensor_msgs::PointCloud2 cloud;
    //projector.projectLaser(*scan_in, cloud);
    //projector.transformLaserScanToPointCloud("axis", *scan_in, cloud, tfListener);




  if(!listener.waitForTransform( scan2_in->header.frame_id, "/axis", scan2_in->header.stamp + ros::Duration().fromSec(scan2_in->ranges.size()*scan2_in->time_increment), ros::Duration(1.0)))
  {
     return;

  }

  sensor_msgs::PointCloud2 cloud2;

  projector.transformLaserScanToPointCloud("axis",*scan2_in, cloud2,listener);

  // Do something with cloud.






    pub_cloud2.publish(cloud2);

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "laser_to_cloud");
    LaserToCloud lasertocloud;
    ros::spin();

}