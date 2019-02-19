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


	ros::Rate loop_rate(0.5);
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
		// msg->header.frame_id = "some_tf_frame";

		// ////////////////////////align two point clouds//////////////////////////
		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		// // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
		transform_1 (0,0) = 0.99668;		transform_1 (0,1) = -0.0494282;		transform_1 (0,2) = 0.0646985;		transform_1 (0,3) = -0.0682588;

		transform_1 (1,0) = 0.0443532;		transform_1 (1,1) =  0.995993;		transform_1 (1,2) = 0.0776559;		transform_1 (1,3) =  -0.0169407;

		transform_1 (2,0) = -0.0682776;		transform_1 (2,1) = -0.0745285;		transform_1 (2,2) = 0.994879;		transform_1 (2,3) = -0.000241279;

	// 0.99668   -0.0494282    0.0646985   -0.0682588
 //   0.0443532     0.995993    0.0776559   -0.0169407
 //  -0.0682776   -0.0745285     0.994879 -0.000241279
 //           0            0            0            1


 //   0.999825   -0.018018  -0.0050312  -0.0230891
 //  0.0183935    0.995885    0.088743 -0.00523142
 // 0.00341153    -0.08882    0.996042  -0.0492599
 //          0           0           0           1


// 		Method #1: using a Matrix4f
//   0.993497  0.0425461   0.105607 -0.0074523
// -0.0557541   0.990532   0.125449  0.0376633
// -0.0992697  -0.130522   0.986463  0.0572808
//          0          0          0          1

	// 0.999824  -0.0175542 -0.00664637  -0.0233176
 //  0.0180657    0.996052   0.0869096 -0.00537353
 //  0.0050945  -0.0870144    0.996194  -0.0503517
 //          0           0           0           1


		// //    (row, column)
		//  Result of calibration method motor-laser
		//   0.998911   0.0276984  -0.0375561 -0.00488246
		// -0.0296548    0.998177  -0.0525759   0.0277208
		//  0.0360313   0.0536325     0.99791  -0.0421599
		//          0           0           0           1

		// 0.97566      0.158887  -0.151135  0.0163712
		// -0.0994964   0.934936   0.340582  0.0599929
  // 		0.195416    -0.317255   0.927988 -0.0980514
  //       0            0          0         1

	// 	Method #1: using a Matrix4f
 //  0.982908   0.173326 -0.0620536   0.010985

 // -0.143305   0.931929   0.333125  0.0971849

 //  0.115569  -0.318539   0.940838 -0.0578071

 //         0          0          0          1


 //  0.975897   0.159029  -0.149449  0.0163069

 // -0.099808   0.934242   0.342388   0.060422

 //  0.194072  -0.319219   0.927597 -0.0973913
 //         0          0          0          1


 //  0.977983   0.188977 -0.0885321  0.0129115

 // -0.150748   0.933099   0.326497  0.0973279

 //   0.14431  -0.305963   0.941043 -0.0733458
 //         0          0          0          1

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