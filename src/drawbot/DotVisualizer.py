#!/usr/bin/env python2

"""Published a visualization of the neato's to rviz"""

from __future__ import division
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, ColorRGBA
from drawbot.srv import GetWaypoints

class DotVisualizer(object):

    def __init__(self):
        super(DotVisualizer, self).__init__()
        self.publisher = rospy.Publisher('/dots', Marker, queue_size=10)
        rospy.wait_for_service('get_waypoints')
        self.waypoints = self.get_waypoints_from_server()
        print self.waypoints

    def get_waypoints_from_server(self):
        try:
            get_waypoints = rospy.ServiceProxy('get_waypoints', GetWaypoints)
            response = get_waypoints()
            return response.points
        except rospy.ServiceException, e:
            print "Service call failed %s" % e

   
    def generate_markers(self, points):
        marker = Marker(
            type=Marker.POINTS,
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='odom'
            ),
            points=points,
            scale=Vector3(0.05, 0.05, 0.05),
            color=ColorRGBA(1.0, 1.0, 0.0, 1.0)
        )
        return marker

    def publish_visualization(self):
        markers = self.generate_markers(self.waypoints)
        self.publisher.publish(markers)

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish_visualization()

if __name__ == '__main__':
    rospy.init_node('DotVisualizer')
    DotVisualizer().run()



