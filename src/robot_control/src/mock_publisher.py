#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import time
import cv2
import numpy as np
from cv_bridge import CvBridge

def mock_publisher():
    rospy.init_node('mock_publisher', anonymous=True)

    # Mock publishers
    control_pub = rospy.Publisher('/recording_control', String, queue_size=10)
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    lidar_pub = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)

    bridge = CvBridge()

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Publish mock control data
        control_pub.publish(String(data='mock_data'))

        # Publish mock image data
        image = np.zeros((480, 640, 3), np.uint8)  # Create a black image
        image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_pub.publish(image_msg)
        depth_pub.publish(image_msg)

        # Publish mock LiDAR data
        lidar_data = PointCloud2()
        lidar_pub.publish(lidar_data)

        rate.sleep()

if __name__ == '__main__':
    try:
        mock_publisher()
    except rospy.ROSInterruptException:
        pass