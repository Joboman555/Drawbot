#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from smach import Sequence
from geometry_msgs.msg import Pose, Point
from drawbot.srv import GetWaypoints
from DrawRow import DrawRow
from Turn import Turn
from GoForward import GoForward
from CarriageReturn import CarriageReturn
from operator import sub
import math

class DotDrawer(object):
    def __init__(self):
        super(DotDrawer, self).__init__()
        rospy.init_node('DotDrawer')

    def get_waypoints_from_server(self):
        try:
            get_waypoints = rospy.ServiceProxy('get_waypoints', GetWaypoints)
            response = get_waypoints()
            return response.waypoints
        except rospy.ServiceException, e:
            print "Service call failed %s" % e

    def extract_rows(self, waypoints):
        """Extracts each row from the list of waypoints"""
        index = 0
        row = 0
        point_rows = []
        while index < len(waypoints):
            dists_in_front = []
            firstPoint = waypoints[index]
            for point in waypoints[index:]:
                if point.location_base_link.x != firstPoint.location_base_link.x:
                    break
                else:
                    index = index + 1
                    dists_in_front.append(point.location_base_link)
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
        # How far did we travel?
        dists_back = []
        for row in rows:
            abs_dists_in_front = [point.y for point in row]
            # Keep track of the farthest distance traveled
            dists_back.append(abs_dists_in_front[-1])
            dists_in_front = self.abs_to_rel(abs_dists_in_front)
            list_of_rows.append(dists_in_front)

        # assume all lines are spaced equally, and that lines are 
        # seperated perfectly on the x axis
        line_spacing = abs(rows[0][0].x - rows[1][0].x)
        print 'Line Spacing: %f' % line_spacing
        
        with sq:
            # Turn to the left initially to start things off.
            Sequence.add(
                'Initial Turn',
                Turn(-1 * math.pi/2),
                transitions={ 
                    'Aborted': 'Aborted'
                }
            )

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
                # If we're on the last line, no need to return
                if i < len(rows) - 1:
                    # Return back to the beginning of row      
                    Sequence.add(
                        'Go Back %d' % i,
                        GoForward(-1 * dists_back[i]),
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
        waypoints = self.get_waypoints_from_server()
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