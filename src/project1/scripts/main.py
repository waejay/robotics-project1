#! /usr/bin/env python

import rospy
import math
import random
from threading import Thread, Lock

from nav_msgs.msg import Odometry       # Determine relative position of robot
from geometry_msgs.msg import Twist     # Control linear/angular velocity
from sensor_msgs.msg import LaserScan   # Calculate distance from objects
from kobuki_msgs.msg import BumperEvent # Detect collisions
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Controller:

    class Coord:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    def __init__(self):
        rospy.init_node("Controller") # initialize node

        self.name = "Controller object"
        self.rate = rospy.Rate(10)    # allow node to spin at 10hz

        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent,
                         self.bumper_callback)


        self.laser = None
        self.odom = None
        self.bumper = None
        self.mutex = Lock()
        self.distanceToWall = 0.3048

        self.prev_coord = self.Coord(0, 0)
        self.total_distance = 0

        # publish to this topic to move the bot
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size =
                        5)

        # Datatype to change motion of bot
        self.vel = Twist()

        # Current state of turtle bot
        self.state = "free"

    def laserscan_callback(self, laser_message):
        self.laser = laser_message

    def odom_callback(self, odom_message):
        self.odom = odom_message

    def bumper_callback(self, bumper_msg):
        self.bumper = bumper_msg

    def distance(self, prev_coord, curr_coord):
        x1, y1 = prev_coord.x, prev_coord.y
        x2, y2 = curr_coord.x, curr_coord.y

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        return dist

    def resetCoord(self):
        self.prev_coord.x = self.odom.pose.pose.position.x
        self.prev_coord.y = self.odom.pose.pose.position.y

    def halt(self):
        self.vel.linear.x = 0.0
        self.vel.angular.x = 0.0

    def rotate_random(self):
        relative_angle = random.uniform(-0.26, 0.26)
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        if relative_angle > 0:
            angular_speed = self.vel.angular.z = 0.5
            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)
        else:
            angular_speed = self.vel.angular.z = -0.5
            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

    def rotate_left(self, rad = 0.45):
        angular_speed = self.vel.angular.z = 0.8
        relative_angle = rad
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        while(current_angle < relative_angle):
            self.velPub.publish(self.vel)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed)*(t1-t0)

    def rotate_right(self, rad = 0.45):
        angular_speed = self.vel.angular.z = -0.8
        relative_angle = rad
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        while(current_angle < relative_angle):
            self.velPub.publish(self.vel)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed)*(t1-t0)

    def rotate_angle(self, angle, speed):
        self.mutex.acquire()
        try:
            angular_speed = speed
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            # setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

            # forcing the robot to stop after rotation
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0
            self.velPub.publish(self.vel)
        finally:
            self.mutex.release()

    def start(self):
        print("Running Controller.start()")

        if not self.bumper:
            self.vel.linear.x = 0.2

        # while the robot hasn't shut down
        while not rospy.is_shutdown():


            # --- Priority 1: Detect collisions ---
            if self.bumper:
                if self.bumper.state == BumperEvent.PRESSED:
                    print("Turtlebot bumped")
                    self.halt()

            # --- Priority 3/4: Obstacles ---
            if self.laser:
                # if robot is 1 ft away from an object (by looking at the middle
                leftLaserValue = self.laser.ranges[250]
                middleLaserValue = self.laser.ranges[180]
                rightLaserValue = self.laser.ranges[110]

                if middleLaserValue <= self.distanceToWall:
                    print("middle laser detected 1.0 fit")
                    self.halt()
                    self.rotate_angle(3.14, -1.5)
                    self.vel.linear.x = 0.2

                if leftLaserValue <= self.distanceToWall:
                    print("left laser detected 1.0 fit")
                    self.rotate_right()
                elif rightLaserValue <= self.distanceToWall:
                    print("right laser detected 1.0 fit")
                    self.rotate_left()
                else:
                    self.vel.linear.x =  0.2
                    self.vel.angular.z = 0.0

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

            # if robot has moved 1 ft, randomly change angle 
            # (within +-15 deg aka +-0.26 rad)
            if self.odom:

                # Initialize starting coordinates
                if self.prev_coord.x == 0 and self.prev_coord.y == 0:
                    self.prev_coord = self.Coord(self.odom.pose.pose.position.x,
                                    self.odom.pose.pose.position.y)


                # TODO: ------ debugging purposes (print to console) --------
                '''
                print(self.odom.pose.pose.position)
                print("Distance increment = %f" % self.distance(self.prev_coord,
                                    self.odom.pose.pose.position))
                print("Total distance (so far): %f" % self.total_distance)
                '''
                # TODO:# ----------------------------------------------------

                if self.distance(self.prev_coord, self.odom.pose.pose.position) >= 0.05:
                    self.total_distance += self.distance(self.prev_coord,
                                    self.odom.pose.pose.position)
                    self.resetCoord()

                # Increment total distance every 0.1 unit distance
                if self.total_distance >= 1.0:
                    self.total_distance = 0
                    self.rotate_random()

            self.velPub.publish(self.vel)    # publish velocity to robot
            self.rate.sleep()
        rospy.spin()                # keep function running continuously until user tells it
                                    # to stop

if __name__ == '__main__':
    controller = Controller()
    controller.start()

