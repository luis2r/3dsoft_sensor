#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "icp_calib");

    ros::NodeHandle nh;

    ros::Publisher pub_cloudICP = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("cloud", 1, false);
    pcl::PointCloud<pcl::PointXYZ> Final1;

    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


    
    // boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > cloud1_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_a");
    // boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > cloud2_subw =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("assembled_cloud_b");
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);  
    // msg->header.frame_id = "some_tf_frame";

  // // Fill in the CloudIn data
  //   cloud_in->width    = 5;
  //   cloud_in->height   = 1;
  //   cloud_in->is_dense = false;
  //   cloud_in->points.resize (cloud_in->width * cloud_in->height);
  //   for (size_t i = 0; i < cloud_in->points.size (); ++i)
  //   {
  //     cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //     cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //     cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  //   }
  //   std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
  //       << std::endl;
  //   for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
  //       cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
  //       cloud_in->points[i].z << std::endl;
  //   *cloud_out = *cloud_in;

  //   std::cout << "size:" << cloud_out->points.size() << std::endl;
  //   for (size_t i = 0; i < cloud_in->points.size (); ++i)
  //     cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

  //   std::cout << "Transformed " << cloud_in->points.size () << " data points:"
  //       << std::endl;
  //   for (size_t i = 0; i < cloud_out->points.size (); ++i)
  //     std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //int iterations = 1;
    //icp.setMaximumIterations (iterations);
    //icp.setMaxCorrespondenceDistance (0.05);
    icp.setMaxCorrespondenceDistance (0.5);

    icp.setInputSource(cloud1_subw);
    icp.setInputTarget(cloud2_subw);

    // PointCloud::Ptr Final(new PointCloud);
    icp.align(Final1);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;






////////////////////////align two point clouds//////////////////////////
  // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_1 = icp.getFinalTransformation();
  // // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  // float theta = M_PI/4; // The angle of rotation in radians
  // transform_1 (0,0) = cos (theta);
  // transform_1 (0,1) = -sin(theta);
  // transform_1 (1,0) = sin (theta);
  // transform_1 (1,1) = cos (theta);
  // //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  // transform_1 (0,3) = 2.5;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;

// Executing the transformation
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud1_subw, *transformed_cloud, transform_1);




cloud_c  = *transformed_cloud;
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