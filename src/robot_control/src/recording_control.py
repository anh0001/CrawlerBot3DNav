#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
import message_filters
import os
from datetime import datetime

# Global variables
recording = False
bag = None
bag_filename = ""
bag_counter = 0
bag_size_limit = 1024 * 1024 * 1024  # 1 GB
buffer_size = 50 * 1024 * 1024  # 50 MB
chunk_size = 10 * 1024 * 1024  # 10 MB
recording_frequency = 10  # 10 Hz

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
    bag = rosbag.Bag(bag_filename, 'w', compression=rosbag.Compression.LZ4, buffer_size=buffer_size, chunk_threshold=chunk_size)
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
    camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber('/camera/color/image_raw/compressed', CompressedImage)
    depth_info_sub = message_filters.Subscriber('/camera/depth/camera_info', CameraInfo)
    depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw/compressed', CompressedImage)
    depth_points_sub = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)
    lidar_points_sub = message_filters.Subscriber('/lidar/points', PointCloud2)

    # Synchronize messages and throttle them to the desired frequency
    ts = message_filters.ApproximateTimeSynchronizer(
        [camera_info_sub, image_sub, depth_info_sub, depth_image_sub, depth_points_sub, lidar_points_sub], 
        queue_size=10, 
        slop=0.1
    )
    ts.registerCallback(lambda *msgs: [save_data_callback(msg, topic_name) for msg, topic_name in zip(msgs, 
                      ['/camera/color/camera_info', '/camera/color/image_raw/compressed', '/camera/depth/camera_info', 
                       '/camera/depth/image_rect_raw/compressed', '/camera/depth/color/points', '/lidar/points'])])

    rospy.spin()

if __name__ == '__main__':
    try:
        recording_control()
    except rospy.ROSInterruptException:
        pass
