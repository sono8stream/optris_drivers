#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2

thermal_pub = rospy.Publisher(
    '/adopter/thermal_mono', Image, queue_size=1)


def thermalCallback(mode):
    thermal_pub.publish(mode)


rospy.init_node('optris_adopter', anonymous=True)

thermal_sub = rospy.Subscriber(
    '/optris/image_mono', Image, thermalCallback, queue_size=1, buff_size=5000000)
rospy.spin()
