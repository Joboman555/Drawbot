#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from smach import Sequence
from geometry_msgs.msg import Pose
from drawbot.srv import GetWaypoints
from DrawRow import DrawRow

class DotDrawer(object):
    def __init__(self):
        super(DotDrawer, self).__init__()
        rospy.init_node('DotDrawer')

    def get_waypoints_from_server(self, pose):
        try:
            get_waypoints = rospy.ServiceProxy('get_waypoints', GetWaypoints)
            response = get_waypoints()
            return response.points
        except rospy.ServiceException, e:
            print "Service call failed %s" % e

    def extract_rows(self, list_of_points):
        """Extracts each row from the list of points"""
        index = 0
        row = 0
        point_rows = []
        while index < len(list_of_points):
            dists_in_front = []
            firstPoint = list_of_points[index]
            for point in list_of_points[index:]:
                if point.x != firstPoint.x:
                    point_rows.append(dists_in_front)
                    break
                else:
                    index = index + 1
                    dists_in_front.append(point)
            row = row + 1
        return point_rows

    def draw_points(self, waypoints):
        """ Draws the given waypoints.
            waypoints : a list of point objects with x, y, and z fields.
        """
        rows = self.extract_rows(waypoints)
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        with sq:
            for i, row in enumerate(rows):
                print 'Added row ' + str(i) + ' to State Machine'
                dists_in_front = [point.y for point in row]            
                Sequence.add(
                    'Draw Row %d' % i,
                    DrawRow(dists_in_front),
                    transitions={ 
                        'Aborted': 'Aborted'
                    }
                )

        # start the state machine
        return sq.execute()

    def run(self):
        rospy.wait_for_service('get_waypoints')
        waypoints = self.get_waypoints_from_server(Pose())
        return self.draw_points(waypoints)


if __name__ == '__main__':
    DotDrawer().run()