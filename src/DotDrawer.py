#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from smach import Sequence
from geometry_msgs.msg import Pose, Point
from drawbot.srv import GetWaypoints
from DrawRow import DrawRow
from CarriageReturn import CarriageReturn
from operator import sub

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
                    break
                else:
                    index = index + 1
                    dists_in_front.append(point)
            point_rows.append(dists_in_front)
            row = row + 1
        return point_rows

    def abs_to_rel(self, abs_dists):
        """ Convert a list of absolute distance to a list of relative distances.
            Ex: abs_to_rel([0, 1, 2, 3, 4]) == [0, 1, 1].
        """
        diff = [0] + abs_dists[0:-1]
        rel_dists = map(sub, abs_dists, diff)
        return rel_dists


    def draw_points(self, waypoints):
        """ Draws the given waypoints.
            waypoints : a list of point objects with x, y, and z fields.
        """
        rows = self.extract_rows(waypoints)
        sq = Sequence(outcomes=['Completed_Successfully', 'Aborted'],
                      connector_outcome='Completed_Successfully')
        
        list_of_rows = []
        for row in rows:
            abs_dists_in_front = [point.y for point in row]
            dists_in_front = self.abs_to_rel(abs_dists_in_front)
            list_of_rows.append(dists_in_front)

        # assume all lines are spaced equally, and that lines are 
        # seperated perfectly on the x axis
        line_spacing = abs(rows[0][0].x - rows[1][0].x)
        print 'Line Spacing: %f' % line_spacing
        
        with sq:
            for i, row in enumerate(rows):
                print 'Added row ' + str(i) + ' to State Machine'
                dists_in_front = list_of_rows[i]

                # Draw a Row       
                Sequence.add(
                    'Draw Row %d' % i,
                    DrawRow(dists_in_front),
                    transitions={ 
                        'Aborted': 'Aborted'
                    }
                )
                # Go to the next line
                Sequence.add(
                    'Carriage Return %d' % i,
                    CarriageReturn(line_spacing),
                    transitions={ 
                        'Aborted': 'Aborted'
                    }
                )

        return sq.execute()

    def run(self):
        rospy.wait_for_service('get_waypoints')
        waypoints = self.get_waypoints_from_server(Pose())
        return self.draw_points(waypoints)

    def test(self):
        points = [Point(x=0.0, y=1.0, z=0.0), Point(x=0.0, y=2.0, z=0.0), 
                  Point(x=1.0, y=1.0, z=0.0), Point(x=1.0, y=3.0, z=0.0)]
        return self.draw_points(points)


if __name__ == '__main__':
    import sys
    if len(sys.argv) <= 1:
        print DotDrawer().run()
    elif sys.argv[1] == '-test':
        print DotDrawer().test()
    else:
        print DotDrawer().run()