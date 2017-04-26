#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from GoForward import GoForward
from Turn import Turn
from Sleep import Sleep
from Draw import Draw

def main():
    rospy.init_node('DotDrawer')

    # create a smach state machine
    sm = smach.StateMachine(outcomes=['give_up'])
    with sm:
        smach.StateMachine.add(
            'Go Forward',
            GoForward(),
            transitions={ 
            # define the transitions that GoForward can go through to other states
                'Completed_Successfully': 'Draw',
                'Aborted': 'give_up'
            }
        )

        smach.StateMachine.add(
            'Draw',
            Draw(),
            transitions={
                'Completed_Successfully': 'Go Forward',
                'Aborted': 'give_up'
            }
        )

    # start the state machine
    outcome = sm.execute()


if __name__ == '__main__':
    main()
