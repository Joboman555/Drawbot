#!/usr/bin/env python2

"""Takes an image path and a pose, returns waypoints for the neato to draw."""

import rospy
import cv2
from geometry_msgs.msg import Point32
from drawbot.srv import GetWaypoints


class ImageProcessor(object):
    def __init__(self, path):
        super(ImageProcessor, self).__init__() 
        rospy.init_node('ImageProcessorServer')
        self.IMAGE = self.preprocess(path, (20, 20))
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

    def handle_get_waypoints(self, req):
        print "Got request for position [%s]" % (req.neatoPosition)
        points = []
        return GetWaypointsResponse(points)

if __name__ == '__main__':
    path = '../images/heart.jpg'
    ImageProcessor(path)

