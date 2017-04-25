#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from smach import Sequence

from GoForward import GoForward
from Sleep import Sleep

class DrawRow(smach.State):
    def __init__(self, dists_in_front,
                       outcomes=['Completed_Successfully', 'Aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.dists_in_front = dists_in_front
        

    def execute(self, userdata):
        return self.run()

    def run(self):
        # create a smach state machine
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        with sq:
            Sequence.add(
                'Go Forward', GoForward(self.dists_in_front[0]), transitions={'Aborted': 'Aborted'}
            )

            Sequence.add(
                'Sleep', Sleep(), transitions={'Aborted': 'Aborted'}
            )
        # start the state machine
        return sq.execute()



if __name__ == '__main__':
    rospy.init_node('DrawRow')
    DrawRow([1.0, 1.0]).run()