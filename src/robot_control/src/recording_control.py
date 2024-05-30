#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String
import os
from datetime import datetime

# Global variables
recording = False
bag = None
bag_filename = ""
bag_counter = 0
bag_size_limit = 1024 * 1024 * 1024  # 1 GB

def control_callback(msg):
    global recording, bag, bag_filename, bag_counter

    if msg.data == 'start' and not recording:
        start_new_bag()
    elif msg.data == 'stop' and recording:
        stop_recording()
    elif msg.data == 'pause' and recording:
        pause_recording()

def start_new_bag():
    global recording, bag, bag_filename, bag_counter
    rospy.loginfo('Starting recording...')
    directory = os.path.join(os.path.dirname(__file__), '../../../rosbag')
    if not os.path.exists(directory):
        os.makedirs(directory)
    bag_counter += 1
    bag_filename = os.path.join(directory, 'recording_{}_part{}.bag'.format(datetime.now().strftime("%Y%m%d_%H%M%S"), bag_counter))
    bag = rosbag.Bag(bag_filename, 'w', compression=rosbag.Compression.LZ4)
    recording = True
    rospy.loginfo('Recording started: {}'.format(bag_filename))

def stop_recording():
    global recording, bag, bag_filename
    rospy.loginfo('Stopping recording...')
    if bag:
        bag.close()
    recording = False
    rospy.loginfo('Recording stopped: {}'.format(bag_filename))

def pause_recording():
    global recording, bag, bag_filename
    rospy.loginfo('Pausing recording...')
    if bag:
        bag.close()
    recording = False
    rospy.loginfo('Recording paused: {}'.format(bag_filename))

def save_data_callback(msg, topic_name):
    global recording, bag
    if recording and bag:
        bag.write(topic_name, msg)
        if bag.size() > bag_size_limit:
            bag.close()
            start_new_bag()

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
