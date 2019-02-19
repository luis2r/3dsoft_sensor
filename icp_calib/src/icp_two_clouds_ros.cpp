#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


#include <pcl/filters/passthrough.h>



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "icp_calib");

    ros::NodeHandle nh;

    ros::Publisher pub_cloudICP = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("kitti_player/hdl64e", 1, false);
    pcl::PointCloud<pcl::PointXYZ> Final1;

    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


    
    // boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_a");
    // boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_b");
    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_a");
    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_b");
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud1");
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud2");

    //std_msgs::StringConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/chatter");
    if (!cloud1_subw->empty() and !cloud2_subw->empty()  )
    {
      std::cout<<"Msg received!"<<std::endl;
    } 
    else 
    {
      std::cout<<"No message!"<<std::endl;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr halfcloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr halfcloud2 (new pcl::PointCloud<pcl::PointXYZ>);  

  //   pcl::PassThrough<pcl::PointXYZ> ptfiltercloud1; // Initializing with true will allow us to extract the removed indices
  //   pcl::PassThrough<pcl::PointXYZ> ptfiltercloud2;
  //   ptfiltercloud1.setInputCloud (cloud1_subw);
  //   ptfiltercloud1.setFilterFieldName ("z");
  //   ptfiltercloud1.setFilterLimits (0.0, 10.0);
  //   ptfiltercloud1.setNegative (false);
  //   ptfiltercloud1.filter (*halfcloud1);
  //   ptfiltercloud1.setNegative (true);
  //   ptfiltercloud1.filter (*halfcloud2);
  //   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  //   icp.setMaxCorrespondenceDistance (0.050);
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




// cloud_c  = *transformed_cloud;
// cloud_b  = *cloud2_subw;
cloud_c  = *cloud1_subw;
cloud_b  = *cloud2_subw;
cloud_c += cloud_b;
  //cloud1_subw->header->frame_id = "axis";
  //pub_cloudICP.publish(Calib_ICP::Final);

    //Calib_ICP calibicp;
    //calibicp.pubICP();
    
    //ros::spin();
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    ROS_INFO("Published Final Cloud with %u points", (uint32_t)(Final1.points.size ())) ;

    // pub_cloudICP.publish (Final1);
    pub_cloudICP.publish (cloud_c);

    ros::spinOnce ();
    loop_rate.sleep ();
  }

}