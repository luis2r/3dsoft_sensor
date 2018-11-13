# 3dsoft_sensor
3D sensor, based on hokuyo urg and stereo camera fusion

roscore

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0

rosrun rosserial_python serial_node.py /dev/ttyUSB0
roslaunch 3d_sensor 3dsensor_description.launch
rosrun hokuyo_node hokuyo_node
rosrun laser_scan_splitter laser_scan_splitter_node
rosrun motor-laser-calib motor-laser-calib_node
roslaunch cloud_scan_assembler.launch
rosrun periodic_snapshotter periodic_snapshotter_node1
rosrun periodic_snapshotter periodic_snapshotter_node2
rosrun icp_calib icp_calib_icp_two_clouds_ros
