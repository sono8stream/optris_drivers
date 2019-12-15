#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2

thermal_pub = rospy.Publisher(
    '/adapter/thermal_mono', Image, queue_size=1)
lidar_pub = rospy.Publisher(
    '/adapter/lidar_points', PointCloud2, queue_size=1)


def thermalCallback(image):
    thermal_pub.publish(image)


def lidarCallback(points):
    lidar_pub.publish(points)


rospy.init_node('optris_adapter', anonymous=True)

thermal_sub = rospy.Subscriber(
    '/optris/image_mono', Image, thermalCallback, queue_size=1, buff_size=5000000)
lidar_sub = rospy.Subscriber(
    '/os1_cloud_node/points', PointCloud2, lidarCallback, queue_size=1, buff_size=5000000)
rospy.spin()
