# 3dsoft_sensor
#3D sensor, based on hokuyo urg and stereo camera fusion

####calibration#####


rosrun rosserial_python serial_node.py /dev/ttyARDUINO

roslaunch 3d_sensor 3dsensor_description.launch

rosrun hokuyo_node hokuyo_node /dev/ttyHokuyo

rosrun laser_scan_splitter laser_scan_splitter_node

rosrun motor-laser-calib motor-laser-calib_node

cd ~/catkin_ws/src/3dsoft_sensor

roslaunch cloud_scan_assembler.launch

rosrun periodic_snapshotter periodic_snapshotter_node1

rosrun periodic_snapshotter periodic_snapshotter_node2

rosrun icp_calib icp_calib_icp_two_clouds_ros

rosrun cv_camera cv_cameraode

cd /home/luis/catkin_ws/src/3dsoft_sensor/launch

roslaunch usb_cam.launch

rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.0245 right:=/camera2/usb_cam2/image_raw left:=/camera1/usb_cam1/image_raw right_camera:=/camera2/usb_cam2 left_camera:=/camera1/usb_cam1

######## Run fusion #########


rosrun rosserial_python serial_node.py /dev/ttyARDUINO

roslaunch 3d_sensor 3dsensor_description.launch

rosrun hokuyo_node hokuyo_node /dev/ttyHokuyo

rosrun laser_scan_splitter laser_scan_splitter_node

rosrun motor-laser-calib motor-laser-calib_node

rosrun icp_calib ensamble_node 

######## stereo #########

roslaunch elas_ros elas_with_logitech.launch
