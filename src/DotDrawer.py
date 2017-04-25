#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from GoForward import GoForward
from Turn import Turn
from Sleep import Sleep
from geometry_msgs.msg import Pose
from drawbot.srv import GetWaypoints


class DotDrawer(object):
    def __init__(self):
        super(DotDrawer, self).__init__()
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
        rospy.init_node('DotDrawer')

        # create a smach state machine
        sm = smach.StateMachine(outcomes=['give_up'])
        sm.userdata.dist_in_front = 1
        with sm:
            smach.StateMachine.add(
                'Go Forward',
                GoForward(),
                transitions={ 
                # define the transitions that GoForward can go through to other states
                    'Completed_Successfully': 'Sleep',
                    'Aborted': 'give_up'
                }
            )

            smach.StateMachine.add(
                'Sleep',
                Sleep(),
                transitions={
                    'Completed_Successfully': 'Go Forward',
                    'Aborted': 'give_up'
                }
            )

        # start the state machine
        outcome = sm.execute()


if __name__ == '__main__':
    DotDrawer().run()