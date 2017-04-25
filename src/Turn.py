#!/usr/bin/env python2

"""Turns the neato a certain amount."""

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import tf
import math

import smach
import rospy

class Turn(smach.State):
    def __init__(self, goal_angle):
        super(Turn, self).__init__(outcomes=['Completed_Successfully', 'Aborted'])
        self.goal_angle = goal_angle
        self.orientation = None

        self.got_first_odom_msg = False

        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.on_shutdown(self.stop)


    def convert_to_euler(self, x, y, z, w):
        quaternion = (x, y, z, w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return np.array([roll, pitch, yaw])

    def stop(self):
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def update_odometry(self, msg):
        current_quat = msg.pose.pose.orientation
        self.orientation = self.convert_to_euler(current_quat.x, current_quat.y, current_quat.z, current_quat.w)

        if not self.got_first_odom_msg:
            self.got_first_odom_msg = True
    
    def get_angle(self):
        return -self.orientation[2]

    def delta_angle(self, a, b):
        return ((b - a) + math.pi) % (math.pi * 2.0) - math.pi

    def rotate(self, angle):
        r = rospy.Rate(50)

        starting_angle = self.get_angle()
        final_angle = starting_angle + angle

        while not rospy.is_shutdown() and self.got_first_odom_msg:
            delta = self.delta_angle(self.get_angle(), final_angle)
            if abs(delta) >= math.pi / 500.0:
                turn_msg = Twist(angular=Vector3(0.0, 0.0, -delta * 1.0))
                self.publisher.publish(turn_msg)
            else:
                self.stop()
                return True
            r.sleep()
        if rospy.is_shutdown():
            return False

    def execute(self, userdata):
        return self.run()

    def run(self):
        r = rospy.Rate(50)

        # Wait for the first odometry position update to come in
        while not self.got_first_odom_msg:
            r.sleep()

        success = self.rotate(self.goal_angle)
        if success:
            return 'Completed_Successfully'
        else:
            return 'Aborted'

if __name__ == '__main__':
    rospy.init_node('Turn')
    Turn(-math.pi/2).run()
