#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from GoForward import GoForward
from Turn import Turn
from Sleep import Sleep
from geometry_msgs.msg import Pose
from drawbot.srv import GetWaypoints
from DrawRow import DrawRow

class DotDrawer(object):
    def __init__(self):
        super(DotDrawer, self).__init__()
        rospy.init_node('DotDrawer')
        rospy.wait_for_service('get_waypoints')
        self.waypoints = self.get_waypoints_from_server(Pose())
        print self.waypoints[0]

    def get_waypoints_from_server(self, pose):
        try:
            get_waypoints = rospy.ServiceProxy('get_waypoints', GetWaypoints)
            response = get_waypoints()
            return response.points
        except rospy.ServiceException, e:
            print "Service call failed %s" % e


    def run(self):

        # create a smach state machine
        sm = smach.StateMachine(outcomes=['give_up'])
        with sm:
            smach.StateMachine.add(
                'Draw Row',
                DrawRow([1.0, 0.5]),
                transitions={ 
                # define the transitions that GoForward can go through to other states
                    'Completed_Successfully': 'Draw Row',
                    'Aborted': 'give_up'
                }
            )

        # start the state machine
        return sm.execute()


if __name__ == '__main__':
    DotDrawer().run()