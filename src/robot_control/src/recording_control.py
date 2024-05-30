#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String
import os
from datetime import datetime

# Global variable to keep track of recording state
recording = False
bag = None
bag_filename = ""

def control_callback(msg):
    global recording, bag, bag_filename

    if msg.data == 'start' and not recording:
        rospy.loginfo('Starting recording...')
        directory = os.path.join(os.path.dirname(__file__), '../../../rosbag')
        bag_filename = os.path.join(directory, 'recording_{}.bag'.format(datetime.now().strftime("%Y%m%d_%H%M%S")))
        bag = rosbag.Bag(bag_filename, 'w')
        recording = True
        rospy.loginfo('Recording started: {}'.format(bag_filename))
    elif msg.data == 'stop' and recording:
        rospy.loginfo('Stopping recording...')
        bag.close()
        recording = False
        rospy.loginfo('Recording stopped: {}'.format(bag_filename))
    elif msg.data == 'pause' and recording:
        rospy.loginfo('Pausing recording...')
        bag.close()
        recording = False
        rospy.loginfo('Recording paused: {}'.format(bag_filename))

def save_data_callback(msg, topic_name):
    global recording, bag
    if recording and bag:
        bag.write(topic_name, msg)

def recording_control():
    rospy.init_node('recording_control', anonymous=True)

    # Subscribe to the control topic
    rospy.Subscriber('/recording_control', String, control_callback)

    # Subscribe to the sensor topics and set up callbacks to save data
    sensor_topics = [
        ('/camera/color/camera_info', 'sensor_msgs/CameraInfo'),
        ('/camera/color/image_raw', 'sensor_msgs/Image'),
        ('/camera/depth/camera_info', 'sensor_msgs/CameraInfo'),
        ('/camera/depth/image_rect_raw', 'sensor_msgs/Image'),
        ('/camera/depth/color/points', 'sensor_msgs/PointCloud2'),
        ('/lidar/points', 'sensor_msgs/PointCloud2')
    ]

    for topic_name, topic_type in sensor_topics:
        rospy.Subscriber(topic_name, rospy.AnyMsg, lambda msg, topic_name=topic_name: save_data_callback(msg, topic_name))

    rospy.spin()

if __name__ == '__main__':
    try:
        recording_control()
    except rospy.ROSInterruptException:
        pass