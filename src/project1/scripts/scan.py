#!/usr/bin/env python

import rospy, sys
from sensor_msgs.msg import LaserScan

def callback(msg):
    console_output_length = sum([len(str(i)) for i in msg.ranges])
    print(msg.ranges)
    print('\b' * console_output_length)
    sys.stdout.flush()
    '''
    ranges = [msg.ranges[639], msg.ranges[540], msg.ranges[360], msg.ranges[180], msg.ranges[0]]

    directions = {
        msg.ranges[639]: "Left",
        msg.ranges[540]: "Leftish",
        msg.ranges[360]: "Front",
        msg.ranges[180]: "Rightish",
        msg.ranges[0]: "Right"
    }

    # Output which direction the laser scans the farthest
    # print directions.get(max(ranges), "null")

    print ranges

    print msg.ranges[0]
    print msg.ranges[360]
    print msg.ranges[639] # Note: in the referred tutorial it says to range up
                          # to 729, but in my case only 639 works; not sure
                          # why
    '''

rospy.init_node('scan_values')
sub = rospy.Subscriber('/laserscan', LaserScan, callback)

# keep python from exiting until this node is stopped
rospy.spin()
