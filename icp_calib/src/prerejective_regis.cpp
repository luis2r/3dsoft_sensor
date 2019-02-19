#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/registration/ndt.h>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "prerejective_regis");

    ros::NodeHandle nh;

    // ros::Publisher pub_cloudICP = nh.advertise<pcl::PointCloud<pcl::PointNormal> > ("cloud_align", 1, false);

    // Point clouds
    PointCloudT::Ptr object (new PointCloudT);
    PointCloudT::Ptr object_aligned (new PointCloudT);
    PointCloudT::Ptr scene (new PointCloudT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    // pcl::PointCloud<pcl::PointXYZ> Final1;
    // pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& object =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud1_halfscan_a");
    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scene=  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> > ("cloud2_halfscan_b");
    // const pcl::PointCloud<pcl::PointNormal>::ConstPtr& a =  ros::topic::waitForMessage<pcl::PointCloud<pcl::PointNormal> > ("cloud1_halfscan_a");
    // const pcl::PointCloud<pcl::PointNormal>::ConstPtr& b =    ros::topic::waitForMessage<pcl::PointCloud<pcl::PointNormal> > ("cloud2_halfscan_b");
    const sensor_msgs::PointCloud2ConstPtr& a=  ros::topic::waitForMessage<sensor_msgs::PointCloud2> ("cloud1_halfscan_a");
    const sensor_msgs::PointCloud2ConstPtr& b =    ros::topic::waitForMessage<sensor_msgs::PointCloud2> ("cloud2_halfscan_b");

    pcl::PCLPointCloud2 pclmsga;
    pcl_conversions::toPCL(*a,pclmsga);

    pcl::PCLPointCloud2 pclmsgb;
    pcl_conversions::toPCL(*b,pclmsgb);

    // pcl_conversions::toPCL(*a, object);
    // pcl_conversions::toPCL(*b, scene);

    pcl::fromPCLPointCloud2(pclmsga,*object);
    pcl::fromPCLPointCloud2(pclmsgb,*scene);


    if (!object->empty() and !scene->empty()  )
    {
      std::cout<<"Msg received!"<<std::endl;
    } 
    else 
    {
      std::cout<<"No message!"<<std::endl;
    }




     // Downsample
    pcl::console::print_highlight ("Downsampling...\n");
    // pcl::VoxelGrid<PointNT> grid;
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.01f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    grid.setInputCloud (scene);
    grid.filter (*scene);

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.2);
    nest.setInputCloud (scene);
    nest.compute (*scene);

    // Estimate features
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.3);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (1); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (3); // Number of nearest features to use
    align.setSimilarityThreshold (0.09); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (0.5 * leaf); // Inlier threshold
    align.setInlierFraction (0.025f); // Required inlier fraction for accepting a pose hypothesis

    {
      pcl::ScopeTime t("Alignment");
      align.align (*object_aligned);
    }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // ROS_INFO("Published Final Cloud with %u points", (uint32_t)(Final1.points.size ())) ;

    // pub_cloudICP.publish (align);
    
    // Show alignment
    // pcl::visualization::PCLVisualizer visu("Alignment");
    // visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    // visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    // visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }




////////////////////////align two point clouds//////////////////////////
  // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  // Eigen::Matrix4f transform_1 = ndt.getFinalTransformation();
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
  // printf ("Method #1: using a Matrix4f\n");
  // std::cout << transform_1 << std::endl;
  //   // Transforming unfiltered, input cloud using found transform.
  //   pcl::transformPointCloud (*cloud1_subw, *output_cloud, ndt.getFinalTransformation ());

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

    // cloud_c  = *output_cloud;
    // cloud_b  = *cloud2_subw;
    // cloud_c += cloud_b;

  ros::Rate loop_rate(4);
  while (nh.ok())
  {


    ros::spinOnce ();
    loop_rate.sleep ();
  }
}