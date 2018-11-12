#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 100
laser_frequency = 40

count = 0
r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []

    for i in range(0, num_readings):
	angle = scan.angle_min + i*scan.angle_increment
	if (i>=40.0 and i<=60.0):
		scan.ranges.append(5/np.cos(angle))  # fake data
	if (i<40.0):
		scan.ranges.append(-1.5/np.sin(angle))  # fake data
		#scan.ranges.append(i/8.0)  # fake data
	if (i>60.0):
		scan.ranges.append(1.5/np.sin(angle))  # fake data		
		#scan.ranges.append(i/12.0)  # fake data

        scan.intensities.append(1)  # fake data

    scan_pub.publish(scan)
    count += 1
    r.sleep()
