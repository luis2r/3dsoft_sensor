#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/stereo_camera_model.h>

//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

typedef struct {
    sensor_msgs::Image msg_img;
    sensor_msgs::CameraInfo info;
    image_transport::CameraPublisher publisher;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> manager;
    std::string url;
    image_geometry::PinholeCameraModel model_;
} w_cam;

class Sync {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
    image_transport::Subscriber sub1_;
    image_transport::Subscriber sub2_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_geometry::StereoCameraModel model_;


public:
    w_cam cam1;
    w_cam cam2;
    ~Sync();
    Sync(ros::NodeHandle nh_, ros::NodeHandle nh_priv_);
    void imageCallback1(const sensor_msgs::ImageConstPtr& raw_msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& raw_msg);
};



int main(int argc, char** argv) {

    ros::init(argc, argv, "fake_sync");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    Sync sync(n, n_priv);
    ROS_INFO("Initializing fake_sync Node...");

    ros::spin();

    return 0;
}




Sync::~Sync() {

}

//###############################################################################################

Sync::Sync(ros::NodeHandle nh_, ros::NodeHandle nh_priv_) :
nh(nh_), nh_priv(nh_priv_) {

    it_.reset(new image_transport::ImageTransport(nh));

    nh_priv.param("narrow/left/camera_info_url", cam1.url, std::string(""));
    nh_priv.param("narrow/right/camera_info_url", cam2.url, std::string(""));

    cam1.publisher = it_->advertiseCamera("narrow/left/image_raw", 1);
    cam2.publisher = it_->advertiseCamera("narrow/right/image_raw", 1);


    ros::NodeHandle nh0("narrow/left/image_raw");
    ros::NodeHandle nh1("narrow/right/image_raw");


    cam1.manager.reset(new camera_info_manager::CameraInfoManager(nh0, "narrow_stereo/left", cam1.url));
    cam2.manager.reset(new camera_info_manager::CameraInfoManager(nh1, "narrow_stereo/right", cam2.url));



    cam1.info = cam1.manager->getCameraInfo();
    cam2.info = cam2.manager->getCameraInfo();

    model_.fromCameraInfo(cam1.info, cam2.info);

    image_transport::TransportHints hints("raw", ros::TransportHints(), nh);
    ROS_INFO("%s", "1");

    sub1_ = it_->subscribe("/left/image_raw", 1, &Sync::imageCallback1, this, hints);
    sub2_ = it_->subscribe("/right/image_raw", 1, &Sync::imageCallback2, this, hints);
    //    sub_ = it_->subscribe("StereoCam/TriImage", 1, &Sync::imageCb, this, hints);

}

//###############################################################################################

void Sync::imageCallback1(const sensor_msgs::Image::ConstPtr& img) {
    //    locked = true;
	// cam1.msg_img.header = img->header;
        // ROS_INFO("%s", "ssr1");


	cam1.msg_img.height= img->height;
	cam1.msg_img.width = img->width;
	cam1.msg_img.encoding = img->encoding;
	cam1.msg_img.is_bigendian = img->is_bigendian;
	cam1.msg_img.step = img->step;
	cam1.msg_img.data = img->data;


    // std_msgs::Header header = img->header;

}

//###############################################################################################

void Sync::imageCallback2(const sensor_msgs::Image::ConstPtr& img) {
    //    locked = true;
        // ROS_INFO("%s", "ssr2");

	cam1.msg_img.header = img->header;

	cam2.msg_img.header = img->header;
	cam2.msg_img.height = img->height;
	cam2.msg_img.width = img->width;
	cam2.msg_img.encoding = img->encoding;
	cam2.msg_img.is_bigendian = img->is_bigendian;
	cam2.msg_img.step = img->step;
	cam2.msg_img.data = img->data;

    std_msgs::Header header = img->header;


    cam1.info.header.frame_id = cam2.info.header.frame_id = "camera";

    //*****************************************************************************
    // publishing 

        cam1.publisher.publish(cam1.msg_img, cam1.info, header.stamp);
        cam2.publisher.publish(cam2.msg_img, cam2.info, header.stamp);


}
//###############################################################################################



