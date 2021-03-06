#! /usr/bin/env python

import rospy
import math
import random
import sys, select, termios, tty
from threading import Thread, Lock

from nav_msgs.msg import Odometry       # Determine relative position of robot
from geometry_msgs.msg import Twist     # Control linear/angular velocity
from sensor_msgs.msg import LaserScan   # Calculate distance from objects
from kobuki_msgs.msg import BumperEvent # Detect collisions

'''
CS4023 - Intelligent Robotics
Project 1: Reactive Robotics using ROS and Gazebo

Collaborators:
    - Chris Jeon
    - Juliana Osgood
    - Sung Jae Yoon
    - John B. Nguyen

Caution: the following code was developed on a personal Ubuntu system and
relies on modifications of the system itself (Hokuyo laser scanner, sudo
commands, etc) and may not work on systems without these dependencies

Copyright: the following code is influenced by the following resources:
    - Github: g40st/ROS_maze_challenge
    - ROS: turtlebot_teleop/turtlebot_teleop_key

Code inspired from these resources are specifically commented. All others are
original or from ROS documentation
'''

class Controller:
    """Driver class for controlling Turtlebot behavior."""

    class Coord:
        """Built-in helper class to track Cartesian coordinates"""
        def __init__(self, x, y):
            self.x = x
            self.y = y

    def __init__(self):

        # Initialize ROS node
        rospy.init_node("Controller")
        self.name = "Controller object"
        self.rate = rospy.Rate(10)      # allow node to 'spin' at 10hz

        # Subscribe to get laser, positional, velocity, and bumper data
        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/mobile_base/commands/velocity', Twist,
                         self.velocity_callback)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent,
                         self.bumper_callback)

        # Publish to Turtlebot's velocity node
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size =
                        5)

        self.laser = None                   # LaserScan data
        self.odom = None                    # Odometry data
        self.bumper = None                  # Bumper data
        self.currVel = None                 # current velocity data
        self.mutex = Lock()                 # locks Turtlebot for strict rotation
        self.distanceToWall = 0.3048        # 0.3048 m = 1 meter
        self.prev_coord = self.Coord(0, 0)  # track Turtlebot's previous coordinate
        self.total_distance = 0             # Turtlebot's total traveled distance
        self.vel = Twist()                  # datatype to modify Turtlebot velocity

    def laserscan_callback(self, laser_message):
        """Retrieves LaserScan topic's data"""
        self.laser = laser_message

    def odom_callback(self, odom_message):
        """Retrieves Odometry topic's data"""
        self.odom = odom_message

    def bumper_callback(self, bumper_msg):
        """Retrieves BumperEvent topic's data"""
        self.bumper = bumper_msg

    def velocity_callback(self, vel_msg):
        """Retrieves Velocity data"""
        self.currVel = vel_msg

    def distance(self, prev_coord, curr_coord):
        """Calculates distance between two cartesian coordinates

        Note: uses distance formula
        """
        x1, y1 = prev_coord.x, prev_coord.y
        x2, y2 = curr_coord.x, curr_coord.y

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        return dist

    def resetCoord(self):
        """Redefines Turtlebot's base coordinates"""
        self.prev_coord.x = self.odom.pose.pose.position.x
        self.prev_coord.y = self.odom.pose.pose.position.y

    def halt(self):
        """Halt Turtlebot's motion"""
        self.vel.linear.x = 0.0
        self.vel.angular.x = 0.0

    def getKey(self):
        """Get pressed key from system"""

        # Note: the following is taken from ROS's turtlebot_teleop_key
        #       script
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def rotate_random(self):
        """Rotates Turtlebot randomly within +-15 deg

        Note: Turtlebot continues linear velocity while
        rotating
        """
        relative_angle = random.uniform(-0.26, 0.26)
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        # Check if angle is positive
        if relative_angle > 0:
            angular_speed = self.vel.angular.z = 0.5

            # Rotate until we get target angle
            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)
        # Otherwise angle is negative
        else:
            angular_speed = self.vel.angular.z = -0.5

            # Rotate until we get target angle
            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

    def rotate_left(self, rad = 0.45):
        """Rotates Turtlebot left

        Note: Turtlebot continues linear velocity while
        rotating
        """
        angular_speed = self.vel.angular.z = 0.8
        relative_angle = rad
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        # Rotate until we get target angle
        while(current_angle < relative_angle):
            self.velPub.publish(self.vel)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed)*(t1-t0)

    def rotate_right(self, rad = 0.45):
        """Rotates Turtlebot right

        Note: Turtlebot continues linear velocity while
        rotating
        """
        angular_speed = self.vel.angular.z = -0.8
        relative_angle = rad
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        # Rotate until we get target angle
        while(current_angle < relative_angle):
            self.velPub.publish(self.vel)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed)*(t1-t0)

    def rotate_angle(self, angle, speed):
        """Rotates Turtlebot a given angle

        Note: Halts Turtlebot's motion during rotation
        """

        # Lock thread to strictly rotate only 
        # (i.e. no forward movement)
        self.mutex.acquire()
        try:
            angular_speed = speed
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

            # Halt Turtlebot after entire rotation
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0
            self.velPub.publish(self.vel)
        finally:

            # Release thread
            self.mutex.release()

    def start(self):
        """Starts Turtlebot's automonous nature and
        initializes behaviors based on states priorities

        """
        print("Running Controller.start()")

        while not rospy.is_shutdown():

            ''' --- Priority 1: Detect collisions --- '''
            if self.bumper:
                # Halt Turtlebot if bumped into something
                if self.bumper.state == BumperEvent.PRESSED:
                    print("Turtlebot bumped")
                    self.halt()

            '''  --- Priority 2: User-controlled movement --- '''
            # Note: the following is taken from ROS's turtlebot_teleop_key
            #       script
            if self.getKey():
                key = self.getKey()
                if key == '\x03':
                    rospy.signal_shutdown("Shutdown signaled via Control-C")
                    break
                elif key == 'j':
                    self.vel.linear.x = 0.2
                    self.vel.angular.z = 2.0
                    self.velPub.publish(self.vel)
                elif key == 'l':
                    self.vel.linear.x = 0.2
                    self.vel.angular.z = -2.0
                    self.velPub.publish(self.vel)
                elif key == ' ':
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                    self.velPub.publish(self.vel)
                    continue


            ''' --- Priority 3/4: Obstacles --- '''
            if self.laser:

                # Get left, mid, and right laser values
                leftLaserValue = self.laser.ranges[250]
                middleLaserValue = self.laser.ranges[180]
                rightLaserValue = self.laser.ranges[110]

                # If laser detects object 1 ft away
                if middleLaserValue <= self.distanceToWall:

                    # Stop and rotate 180 degrees
                    self.halt()
                    self.rotate_angle(3.14, -1.5)

                    # Then continue moving
                    self.vel.linear.x = 0.2

                # If laser detects object towards the left
                if leftLaserValue <= self.distanceToWall:

                    # Reflexively rotate right while moving forward
                    self.rotate_right()

                # If laser detects object towards the right
                elif rightLaserValue <= self.distanceToWall:

                    # Reflexively rotate left while moving forward
                    self.rotate_left()

                # If no object ahead, move forward
                else:
                    self.vel.linear.x =  0.2
                    self.vel.angular.z = 0.0

            ''' --- Priority 5: Random Rotations --- '''
            if self.odom:

                # Initialize starting coordinates
                if self.prev_coord.x == 0 and self.prev_coord.y == 0:
                    self.prev_coord = self.Coord(self.odom.pose.pose.position.x,
                                    self.odom.pose.pose.position.y)

                # If incremental movement is at least 0.05 m
                if self.distance(self.prev_coord, self.odom.pose.pose.position) >= 0.05:

                    # Increment total distance by that 0.05 m
                    self.total_distance += self.distance(self.prev_coord,
                                    self.odom.pose.pose.position)
                    self.resetCoord()

                # If Turtlebot travels at least 1 ft/0.3048 meters
                if self.total_distance >= 0.3048:

                    # Reset total distance traveled and randomly rotate
                    self.total_distance = 0
                    self.rotate_random()

            ''' --- Priority 6: Drive forward --- '''
            # Move forward 
            if not self.getKey() == 'j' or not self.getKey() == 'l':
                self.vel.linear.x = 0.2

            self.velPub.publish(self.vel)    # Publish our new velocity data to
                                             # the Turtlebot
            self.rate.sleep()
            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.spin()                         # Keep function running continuously until user tells it
                                             # to stop

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    controller = Controller()
    controller.start()

