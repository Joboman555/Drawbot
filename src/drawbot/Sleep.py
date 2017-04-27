#!/usr/bin/env python2

"""Sleeps the neato for a ceratin time."""

import smach
import rospy

class Sleep(smach.State):

    def __init__(self, outcomes=['Completed_Successfully', 'Aborted']):
        super(Sleep, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        return self.run()

    def run(self):
        try:
            rospy.sleep(0.1)
        except:
            return 'Aborted'
        else:
            return 'Completed_Successfully'

if __name__ == '__main__':
    rospy.init_node('Sleep')
    Sleep().run()
