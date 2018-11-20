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
	ros::Publisher  pub_out_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("cloud", 1, false);

	ros::Rate loop_rate(0.5);
	while (nh.ok())
	{
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_a");
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_b");
		//std_msgs::StringConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/chatter");
		if (!cloud1_subw->empty() and !cloud2_subw->empty()  )
		{
			std::cout<<"Msg received!"<<std::endl;
		} 
		else 
		{
			std::cout<<"No message!"<<std::endl;
		}
		// msg->header.frame_id = "some_tf_frame";

		// ////////////////////////align two point clouds//////////////////////////
		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		// // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
		transform_1 (0,0) = 0.998911;
		transform_1 (0,1) = 0.0276984;
		transform_1 (0,2) = -0.0375561;
		transform_1 (0,3) = -0.00488246;

		transform_1 (1,0) = -0.0296548;
		transform_1 (1,1) =  0.998177;
		transform_1 (1,2) =  -0.0525759;
		transform_1 (1,3) =  0.0277208;

		transform_1 (2,0) = 0.0360313;
		transform_1 (2,1) =  0.0536325;
		transform_1 (2,2) = 0.99791;
		transform_1 (2,2) = -0.0421599;
		// //    (row, column)
		//  Result of calibration method motor-laser
		//   0.998911   0.0276984  -0.0375561 -0.00488246
		// -0.0296548    0.998177  -0.0525759   0.0277208
		//  0.0360313   0.0536325     0.99791  -0.0421599
		//          0           0           0           1

		// Print the transformation
		printf ("Method #1: using a Matrix4f\n");
		std::cout << transform_1 << std::endl;

		// Executing the transformation
		pcl::transformPointCloud (*cloud1_subw, *transformed_cloud, transform_1);

		cloud_c  = *transformed_cloud;
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