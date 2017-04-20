#!/usr/bin/env python2

"""Takes an image path and a pose, returns waypoints for the neato to draw."""

from __future__ import division
import rospy
import cv2
from geometry_msgs.msg import Point
from drawbot.srv import GetWaypoints, GetWaypointsResponse
import numpy as np
import sys


class ImageProcessor(object):
    def __init__(self, path):
        super(ImageProcessor, self).__init__() 
        rospy.init_node('ImageProcessorServer')
        self.IMAGE = self.preprocess(path, final_size=(40, 40))
        s = rospy.Service('get_waypoints', GetWaypoints, self.handle_get_waypoints)
        print "Ready to give waypoints"
        print self.IMAGE
        rospy.spin()

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
        return points

    def handle_get_waypoints(self, req):
        print "Got request for position [%s]" % (req.neatoPosition)
        points = self.generate_points(self.IMAGE, (3,3))
        return GetWaypointsResponse(points)

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        # We didn't get a path passed in
        print 'ERROR: Must pass in a path variable!'
    else:
        path = sys.argv[1]
        print path
        ImageProcessor(path)

