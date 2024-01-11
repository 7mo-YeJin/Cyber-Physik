#!/usr/bin/env python
import sys
import time
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Duration

def plot_x(msg):
    global last_time
    if (time.time() - last_time >0.1):
        plt.clf()
        plt.plot(msg.ranges)
        plt.draw()
        plt.pause(0.01)
        last_time = time.time()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        topic_name = "/scan"
    last_time = time.time()
    rospy.init_node("plotter")
    rospy.Subscriber(topic_name, LaserScan, plot_x)
    plt.ion()
    rospy.spin()
