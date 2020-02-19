#! /usr/bin/env python

import rospy
import sys

from nav_msgs.msg import Odometry       # Determine relative position of robot
from geometry_msgs.msg import Twist     # Control linear/angular velocity
from sensor_msgs.msg import LaserScan   # Calculate distance from objects


class Controller:

    def __init__(self):
        self.name = "Controller object"
        self.rate = rospy.Rate(10)      # TODO: figure out what this does LMAO

        # subscribe to laserscan and odometry to
        # get laser data and positional data
        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback) 
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # publish to this topic to move the bot
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size =
                        5)

        # Datatype to change motion of bot
        self.vel = Twist()

        # Current state of turtle bot
        # free = no obstacles nearby
        # asymmetricDetected = ...
        # symmetricDetected = ...
        self.state = "free"

    def start(self):

        # initialize node
        rospy.init_node("Controller")

        # while the robot hasn't shut down

            # take movement directions from user (user input movement)
 
            # if robot has moved 1 ft, randomly change angle (within +-15 deg)

            # if robot is 1 ft away from an object (by looking at the middle
            # laser)
                # check robot state (i.e. "symmetricDetected",
                # "asymmetricDetected")

                # if symmetricDetected:
                    # halt robot motion
                    # change angle 180 deg
                    # change state back to free state
                    # continue

                # if asymmetricDetected:
                    # halt robot
                    # move away from closer object (not towards the second
                        # object
                    # change state back to free state
                    # continue
        
        self.velPub.publish(vel)    # publish velocity to robot
        rospy.spin()                # keep function running continuously until user tells it
                                    # to stop

if __name__ == '__main__':
    controller = Controller()
    controller.start()

    print("End program.")

