#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>

// ros::NodeHandle nh;
ros::Publisher cloud1_halfscan_a ;
ros::Publisher cloud1_halfscan_b;
ros::Publisher cloud2_halfscan_a;
ros::Publisher cloud2_halfscan_b;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_a(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_b(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_a(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_b(new pcl::PointCloud<pcl::PointXYZ>);

// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_a;
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_b;
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_a;
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_b;
static sensor_msgs::PointCloud2 scloud1_halfscan_a;
static sensor_msgs::PointCloud2 scloud1_halfscan_b;
static sensor_msgs::PointCloud2 scloud2_halfscan_a;
static sensor_msgs::PointCloud2 scloud2_halfscan_b;

static pcl::PCLPointCloud2 pcl1_half_a;
static pcl::PCLPointCloud2 pcl1_half_b;
static pcl::PCLPointCloud2 pcl2_half_a;
static pcl::PCLPointCloud2 pcl2_half_b;

bool firstcloud_1;
bool firstcloud_2;
float angle;

void pointCloud1Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
// void pointCloud1Callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{ // ROS_INFO("recieved points");
  // pcl::PCLPointCloud2 pcl_pc;
  // pcl_conversions::toPCL(*msg, pcl_pc);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(pcl_pc,*cloud);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  pcl::PCLPointCloud2 pclmsg;
  pcl_conversions::toPCL(*msg,pclmsg);

  if(firstcloud_1 == true){
    ROS_INFO("I heard: [%f]", angle);
    pcl1_half_a = pclmsg;
    pcl1_half_b = pclmsg;
    firstcloud_1 =false;
  }
  else
  {
    if(angle>0 and angle<180)
    {
      pcl::concatenatePointCloud (pcl1_half_a, pclmsg, pcl1_half_a);
      if(angle>170 and angle<180)
      {
          pcl_conversions::fromPCL(pcl1_half_a, scloud1_halfscan_a);
          cloud1_halfscan_a.publish (scloud1_halfscan_a);
      }
    }
    else{
      pcl::concatenatePointCloud (pcl1_half_b, pclmsg, pcl1_half_b);
      if(angle>350 and angle<360)
      {
          pcl_conversions::fromPCL(pcl1_half_b, scloud1_halfscan_b);
          cloud1_halfscan_b.publish (scloud1_halfscan_b);      }
    }
  }
}



void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 pclmsg;
  pcl_conversions::toPCL(*msg,pclmsg);

  if(firstcloud_2 == true){
    ROS_INFO("I heard2: [%f]", angle);
    pcl2_half_a = pclmsg;
    pcl2_half_b = pclmsg;
    firstcloud_2 =false;
  }
  else
  {
    if(angle>0 and angle<180)
    {
      pcl::concatenatePointCloud (pcl2_half_a, pclmsg, pcl2_half_a);
      if(angle>179 and angle<180)
      {
          pcl_conversions::fromPCL(pcl2_half_a, scloud2_halfscan_a);
          cloud2_halfscan_a.publish (scloud2_halfscan_a);
      }
    }
    else{
      pcl::concatenatePointCloud (pcl2_half_b, pclmsg, pcl2_half_b);
      if(angle>359 and angle<360)
      {
          pcl_conversions::fromPCL(pcl2_half_b, scloud2_halfscan_b);
          cloud2_halfscan_b.publish (scloud2_halfscan_b);      }
    }
  }
}


void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
  angle = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_sub_pcl");

  ros::NodeHandle nh;
  cloud1_halfscan_a = nh.advertise<sensor_msgs::PointCloud2> ("cloud1_halfscan_a", 1, false);
  cloud1_halfscan_b = nh.advertise<sensor_msgs::PointCloud2> ("cloud1_halfscan_b", 1, false);
  cloud2_halfscan_a = nh.advertise<sensor_msgs::PointCloud2> ("cloud2_halfscan_a", 1, false);
  cloud2_halfscan_b = nh.advertise<sensor_msgs::PointCloud2> ("cloud2_halfscan_b", 1, false);

  ros::Subscriber sub = nh.subscribe("angle", 1, angleCallback);
  ros::Subscriber sub1 = nh.subscribe ("cloud1", 1, pointCloud1Callback);
  ros::Subscriber sub2 = nh.subscribe ("cloud2", 1, pointCloud2Callback);

  // ros::Subscriber sub2 = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("cloud2", 1000, pointCloud1Callback);
// // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// scloud1_halfscan_a = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
// scloud1_halfscan_b = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
// scloud2_halfscan_a = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
// scloud2_halfscan_b = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_a(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud1_halfscan_b(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_a(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr scloud2_halfscan_b(new pcl::PointCloud<pcl::PointXYZ>);
firstcloud_1 = true;
firstcloud_2 = true;
angle = 0.0;

  ros::spin();
  return 0;
}

  // PointCloud::Ptr msg_out (new PointCloud);
  // msg_out->header.frame_id = "some_tf_frame";
  // msg_out->height = msg->width = 1;
  // msg_out->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
  // pcl_conversions::toPCL(ros::Time::now(), msg_out->header.stamp);
  // pub.publish (msg_out);
