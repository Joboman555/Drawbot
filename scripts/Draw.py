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

        while not rospy.is_shutdown():
            self.pub.publish(Byte(data=60))
            r.sleep()
            self.pub.publish(Byte(data=68))
            r.sleep()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('Draw')
    Draw().run()
