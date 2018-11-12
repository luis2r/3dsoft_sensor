#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

//#include <tf/transform_broadcaster.h>
ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;
double swivel=0;
void chatterCallback(const std_msgs::Float32::ConstPtr& angle)
{

	//swivel = angle->data;
    swivel = (angle->data*M_PI)/180;

	ROS_INFO("I heard: [%f]", angle->data);


	        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] ="swivel";
        //joint_state.position[0] = swivel;
        joint_state.position[0] = swivel;
        ROS_INFO("I heard: [%f]", swivel);

        //joint_state.name[1] ="tilt";
        //joint_state.position[1] = tilt;
        //joint_state.name[2] ="periscope";
        //joint_state.position[2] = height;


        // update transform
        // (moving in a circle with radius=2)
        //odom_trans.header.stamp = ros::Time::now();
        //odom_trans.transform.translation.x = 0.10;
        //odom_trans.transform.translation.y = 0.10;
        //odom_trans.transform.translation.x = cos(angle)*2;
        //odom_trans.transform.translation.y = sin(angle)*2;
        //odom_trans.transform.translation.z = .1;
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(10+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber sub = n.subscribe("angle", 1000, chatterCallback);
    //ros::Subscriber sub = n.subscribe("sick_roll", 1000, chatterCallback);

    //tf::TransformBroadcaster broadcaster;
    //ros::Rate loop_rate(30);

    //const double degree = M_PI/180;

    // robot state
    //double angle, swivel=0;

    // message declarations
    //geometry_msgs::TransformStamped odom_trans;
    // sensor_msgs::JointState joint_state;
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = "axis";

    // while (ros::ok()) {


    // }


    ros::spin();
        return 0;
}
