#!/usr/bin/env python2

"""Takes an image path and a pose, returns waypoints for the neato to draw."""

from __future__ import division
import rospy
import cv2
from geometry_msgs.msg import Point
from drawbot.msg import Waypoint
from nav_msgs.msg import Odometry
from drawbot.srv import GetWaypoints, GetWaypointsResponse
import numpy as np
import sys
import tf


class ImageProcessor(object):
    def __init__(self, path):
        super(ImageProcessor, self).__init__() 
        rospy.init_node('ImageProcessorServer')

        self.starting_position = None
        self.starting_orientation = None
        self.got_first_odom_msg = False

        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        IMAGE = self.preprocess(path, final_size=(3, 3))
        s = rospy.Service('get_waypoints', GetWaypoints, self.handle_get_waypoints)
        print "Ready to give waypoints"
        print IMAGE
        r = rospy.Rate(50)
        while not self.got_first_odom_msg:
            r.sleep()
        self.bl_coords = self.generate_base_link_points(IMAGE, (0.5,0.5))
        self.odom_coords = self.generate_odom_points(IMAGE, (0.5,0.5))
        print self.starting_position
        print self.starting_orientation
        rospy.spin()

    def update_odometry(self, msg):
        if not self.got_first_odom_msg:
            pos = msg.pose.pose.position
            self.starting_position = np.array([pos.x, pos.y, pos.z])
            
            current_quat = msg.pose.pose.orientation
            self.starting_orientation = self.convert_to_euler(current_quat.x, current_quat.y, current_quat.z, current_quat.w)
            
            self.got_first_odom_msg = True

    def convert_to_euler(self, x, y, z, w):
        quaternion = (x, y, z, w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return np.array([roll, pitch, yaw])

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

    # def extract_rows(self, list_of_points):
    #     """Extracts each row from the list of points"""
    #     index = 0
    #     row = 0
    #     point_rows = []
    #     while index < len(list_of_points):
    #         dists_in_front = []
    #         firstPoint = list_of_points[index]
    #         for point in list_of_points[index:]:
    #             if point.x != firstPoint.x:
    #                 break
    #             else:
    #                 index = index + 1
    #                 dists_in_front.append(point)
    #         point_rows.append(dists_in_front)
    #         row = row + 1
    #     return point_rows

    def generate_base_link_points(self, binary_image, (x_scale, y_scale)):
        # Finds coordinates where the image is not 0
        x_y_coords = np.argwhere(binary_image).astype(float)
        scaled_x_y_coords = self.scale(x_y_coords, (x_scale, y_scale))
        zs = np.zeros((x_y_coords.shape[0], 1))
        # We need to append x, y, and zs together
        points_coords = np.hstack((scaled_x_y_coords, zs))
        return points_coords

    def generate_odom_points(self, binary_image, (x_scale, y_scale)):
        points_coords = self.generate_base_link_points(binary_image, (x_scale, y_scale))
        points_coords_transformed = points_coords + self.starting_position
        return points_coords_transformed

    def generate_waypoints(self, odom_points, base_link_points):
        """ Creates waypoints from two arrays of points.
            odom_points: location of waypoints in odom frame.
            base_link_points: location of waypoitnts in base_link frame."""
        # The second dimension must be 3
        assert odom_points.shape[1] == 3
        assert base_link_points.shape[1] == 3

        waypoints = []
        current_id = 0
        row = 0
        for array_row in np.hstack((odom_points, base_link_points)):
            col = 0
            odom_point = Point(x=array_row[0], y=array_row[1], z=array_row[2])
            # THIS IS NOT FINAL - just for testing
            base_link_point = Point(x=array_row[3], y=array_row[4], z=array_row[5])
            print 
            waypoint = Waypoint(id=current_id, row=row, col=col, 
                location_odom=odom_point, location_base_link=base_link_point)
            waypoints.append(waypoint)
            current_id = current_id + 1
            col = col + 1
        return waypoints

    def handle_get_waypoints(self, req):
        print "Got request for waypoints "
        
        waypoints = self.generate_waypoints(self.odom_coords, self.bl_coords)
        return GetWaypointsResponse(waypoints)

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        # We didn't get a path passed in
        print 'ERROR: Must pass in a path variable!'
    else:
        path = sys.argv[1]
        print path
        ImageProcessor(path)

