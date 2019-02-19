#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "ensamble_cloud");

	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ> Final1;
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	ros::Publisher  pub_out_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("kitti_player/hdl64e", 1, false);


	ros::Rate loop_rate(10);
	while (nh.ok())
	{
		// const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud1_halfscan_a");
		// const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud2_halfscan_b");
				const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud1");
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud2");
		//std_msgs::StringConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/chatter");
		if (!cloud1_subw->empty() and !cloud2_subw->empty()  )
		{
			std::cout<<"Msg received a!"<<std::endl;
		} 
		else 
		{
			std::cout<<"No message b!"<<std::endl;
		}


		cloud_c  = *cloud1_subw;
		cloud_b  = *cloud2_subw;
		cloud_c += cloud_b;
		//cloud1_subw->header->frame_id = "axis";
		//pub_cloudICP.publish(Calib_ICP::Final);

		//ros::spin();

		ROS_INFO("Published Final Cloud with %u points", (uint32_t)(Final1.points.size ())) ;
		pub_out_cloud.publish (cloud_c);
		ros::spinOnce ();
		loop_rate.sleep ();
	}
}