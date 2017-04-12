#!/usr/bin/env python2

"""Displays an image visualization to rviz."""

from __future__ import division
import numpy as np
import smach
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class ImageVisualizer(smach.State):

    def __init__(self, outcomes=['Done']):
        super(ImageVisualizer, self).__init__(outcomes=outcomes)

        self.dots_msg_sent = False

        self.publisher = rospy.Publisher('/dots', Marker, queue_size=10)
        rospy.Subscriber('/dots', Marker, self.get_dots_msg)

    def get_dots_msg(self, msg):
        if not self.dots_msg_sent:
            self.dots_msg_sent = True

    def scale(self, coords, (x_scale, y_scale)):
        """Scale an array of coordinates so that it's maximum is x_scale, yscale"""
        maxs = np.amax(coords, axis=0)
        scaled = coords * np.array([x_scale, y_scale]) / maxs
        return scaled

    def generate_points(self, binary_image, (x_scale, y_scale)):
        # Finds coordinates where the image is not 0
        x_y_coords = np.argwhere(binary_image).astype(float)
        scaled_x_y_coords = self.scale(x_y_coords, (x_scale, y_scale))
        zs = np.zeros((x_y_coords.shape[0], 1))
        # We need to append x, y, and zs together
        points_coords = np.hstack((scaled_x_y_coords, zs))
        points = []
        for row in points_coords:
            point = Point(x=row[0], y=row[1], z=row[2])
            points.append(point)
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

    def execute(self, userdata):
        return self.run()

    def run(self):
        r = rospy.Rate(50)
        image = np.ones((5,5))

        # Keep sending message until it is sent successfully.
        while not rospy.is_shutdown(): #and not self.dots_msg_sent:
            points = self.generate_points(image, (3,4))
            self.publisher.publish(points)

        return 'Done'

if __name__ == '__main__':
    rospy.init_node('ImageVisualizer')
    ImageVisualizer().run()
