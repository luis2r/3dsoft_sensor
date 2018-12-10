#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>


#include <pcl/registration/ndt.h>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "icp_calib");

    ros::NodeHandle nh;

    ros::Publisher pub_cloudICP = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("cloud_ndf", 1, false);
    pcl::PointCloud<pcl::PointXYZ> Final1;

    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

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
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);  
    //   // Loading first scan of room.
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // // Filtering input scan to roughly 10% of original size to increase speed of registration.
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    // approximate_voxel_filter.setInputCloud (input_cloud);
    // approximate_voxel_filter.filter (*filtered_cloud);
    // std::cout << "Filtered cloud contains " << filtered_cloud->size ()
    //           << " data points from room_scan2.pcd" << std::endl;

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (cloud1_subw);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (cloud2_subw);

    // Set initial alignment estimate found using robot odometry.
    //Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    //Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    //Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity ();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*cloud1_subw, *output_cloud, ndt.getFinalTransformation ());

    // Saving transformed input cloud.
    // pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

    //   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //   icp.setMaxCorrespondenceDistance (0.5);
    //   icp.setInputSource(cloud1_subw);
    //   icp.setInputTarget(cloud2_subw);
    //   icp.align(Final1);
    //   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    //   icp.getFitnessScore() << std::endl;
    //   std::cout << icp.getFinalTransformation() << std::endl;

    // Eigen::Matrix4f transform_1 = icp.getFinalTransformation();

    // printf ("Method #1: using a Matrix4f\n");
    // std::cout << transform_1 << std::endl;

    // pcl::transformPointCloud (*cloud1_subw, *transformed_cloud, transform_1);

    cloud_c  = *output_cloud;
    cloud_b  = *cloud2_subw;
    cloud_c += cloud_b;

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    ROS_INFO("Published Final Cloud with %u points", (uint32_t)(Final1.points.size ())) ;

    pub_cloudICP.publish (cloud_c);

    ros::spinOnce ();
    loop_rate.sleep ();
  }
}