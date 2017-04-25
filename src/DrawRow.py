#!/usr/bin/env python2

import rospy
import smach
import smach_ros

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
        sm = smach.StateMachine(outcomes=['Completed_Successfully', 'Aborted'])
        sm.userdata.dist_in_front = dists_in_front[0]
        with sm:
            smach.StateMachine.add(
                'Go Forward',
                GoForward(),
                transitions={ 
                # define the transitions that GoForward can go through to other states
                    'Completed_Successfully': 'Sleep',
                    'Aborted': 'Aborted'
                }
            )

            smach.StateMachine.add(
                'Sleep',
                Sleep(),
                transitions={
                    'Completed_Successfully': 'Completed_Successfully',
                    'Aborted': 'Aborted'
                }
            )
        # start the state machine
        return sm.execute()



if __name__ == '__main__':
    rospy.init_node('DrawRow')
    DrawRow().run()