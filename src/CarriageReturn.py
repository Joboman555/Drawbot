#!/usr/bin/env python2

"""After a row has been drawn, this state moves the neato down to the next row"""

import rospy
import smach
from smach import Sequence

from GoForward import GoForward
from Turn import Turn
import math

class CarriageReturn(smach.State):
    def __init__(self, line_spacing):
        smach.State.__init__(self, outcomes=['Completed_Successfully', 'Aborted'])
        self.line_spacing = line_spacing

    def execute(self, userdata):
        return self.run()

    def run(self):
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        with sq:
            # First turn
            Sequence.add(
                'Turn 1', Turn(math.pi/2), transitions={'Aborted': 'Aborted'}
            )
            # Go forward a  bit
            Sequence.add(
                'Go Forward', GoForward(self.line_spacing), transitions={'Aborted': 'Aborted'}
            )
            # Turn again
            Sequence.add(
                'Turn 2', Turn(-1 * math.pi/2), transitions={'Aborted': 'Aborted'}
            )
        # start the state machine
        return sq.execute()


if __name__ == '__main__':
    rospy.init_node('CarriageReturn')
    CarriageReturn(0.1).run()