#!/usr/bin/env python2

"""Displays an image visualization to rviz."""

from __future__ import division
import numpy as np
import smach
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import cv2


class DotMatrix(object):

    def __init__(self, path):
        super(DotMatrix, self).__init__()
        self.IMAGE_PATH = path
        self.publisher = rospy.Publisher('/dots', Marker, queue_size=10)

    def preprocess(self, image_path, final_size):
        """Takes in an image path, and returns a processed numpy array."""
        image = cv2.imread(image_path, 1)
        resized_image = cv2.resize(image, final_size)
        image_gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        ret , binary_image= cv2.threshold(image_gray,127,255,cv2.THRESH_BINARY)
        # binary_image = cv2.adaptiveThreshold(image_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        #             cv2.THRESH_BINARY,11,2)
        return binary_image


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

    def publish_visualization(self):
        bw_image = self.preprocess(self.IMAGE_PATH, (30, 30))
        points = self.generate_points(bw_image, (3,2))
        self.publisher.publish(points)

    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.publish_visualization()
            

if __name__ == '__main__':
    rospy.init_node('DotMatrix')
    DotMatrix(path='../images/heart.jpg').run()

