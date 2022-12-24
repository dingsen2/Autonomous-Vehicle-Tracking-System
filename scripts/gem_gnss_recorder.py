#!/usr/bin/env python3
from __future__ import print_function

# Python Headers
import os
import cv2 
import csv
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import tf
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM Sensor Headers
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu, NavSatFix
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva, NovatelCorrectedImuData

csv_file  = 'eight.csv'

class GNSSImage(object):
    
    def __init__(self):

        self.rate = rospy.Rate(15)

        # Subscribe information from sensors
        self.lat      = 0
        self.lon      = 0
        self.heading  = 0
        self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.olat       = 40.0928563
        self.olon       = -88.2359994

        self.offset     = 0.46 # meters

    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude
        self.lon     = inspva_msg.longitude
        self.heading = inspva_msg.azimuth    
    
    def heading_to_yaw(self, heading_curr):
        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y    
    
    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)
        #print(round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4))

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)


    def start_gi(self):
        while not rospy.is_shutdown():
            
            try:
                # Save lat, lon, heading to CSV
                with open(csv_file, 'a', encoding='UTF8') as f:
                    writer = csv.writer(f)

                    x, y, yaw = self.get_gem_state()

                    data = [x, y, round(self.heading, 3)]

                    print(data)
                    writer.writerow(data)

            except:
                rospy.logerr("CSV Error")

            self.rate.sleep()


def main():

    rospy.init_node('gem_gnss_recorder_node', anonymous=True)

    gi = GNSSImage()

    try:
    	gi.start_gi()
    except KeyboardInterrupt:
        print ("Shutting down gnss recorder node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

