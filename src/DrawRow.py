#!/usr/bin/env python2

"""Draws a row of pixels"""

import rospy
import smach
from smach import Sequence

from GoForward import GoForward
from Sleep import Sleep

class DrawRow(smach.State):
    def __init__(self, dists_in_front):
        smach.State.__init__(self, outcomes=['Completed_Successfully', 'Aborted'])
        self.dists_in_front = dists_in_front

    def execute(self, userdata):
        return self.run()

    def run(self):
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        with sq:
            # Iterate through each dist in front, going that dist
            # and then drawing a dot.
            for i, dist in enumerate(self.dists_in_front):
                go_fwd_lbl = 'Go Forward %d' % i
                sleep_lbl = 'Sleep %d' % i
                Sequence.add(
                    go_fwd_lbl, GoForward(dist), transitions={'Aborted': 'Aborted'}
                )
                Sequence.add(
                    sleep_lbl, Sleep(), transitions={'Aborted': 'Aborted'}
                )
        # start the state machine
        return sq.execute()


if __name__ == '__main__':
    rospy.init_node('DrawRow')
    DrawRow([1.0, 0.5]).run()