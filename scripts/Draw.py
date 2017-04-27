#!/usr/bin/env python2

"""Sleeps the neato for a ceratin time."""

import smach
import rospy
from std_msgs.msg import Byte

class Draw(smach.State):

    def __init__(self, outcomes=['Completed_Successfully', 'Aborted']):
        super(Draw, self).__init__(outcomes=outcomes)
        self.pub = rospy.Publisher('/pen', Byte, queue_size=10)

    def execute(self, userdata):
        return self.run()

    def run(self):
        r = rospy.Rate(4)

        for i in range(3):
            self.pub.publish(Byte(data=65))
            r.sleep()
        for i in range(3):
            self.pub.publish(Byte(data=80))
            r.sleep()
        return 'Completed_Successfully'

if __name__ == '__main__':
    rospy.init_node('Draw')
    Draw().run()
