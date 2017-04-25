#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from smach import Sequence

from GoForward import GoForward
from Sleep import Sleep

class DrawRow(smach.State):
    def __init__(self, outcomes=['Completed_Successfully', 'Aborted'], 
                       input_keys=['dists_in_front']):
        smach.State.__init__(self, outcomes=outcomes,
                                   input_keys=input_keys)
        

    def execute(self, userdata):
        return self.run(userdata.dists_in_front)

    def run(self, dists_in_front = [1.0, 1.0]):
        # create a smach state machine
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        with sq:
            Sequence.add(
                'Go Forward', GoForward(dists_in_front[0]), transitions={'Aborted': 'Aborted'}
            )

            Sequence.add(
                'Sleep', Sleep(), transitions={'Aborted': 'Aborted'}
            )
        # start the state machine
        return sq.execute()



if __name__ == '__main__':
    rospy.init_node('DrawRow')
    DrawRow().run()