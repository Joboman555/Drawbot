#!/usr/bin/env python2

"""Goes forward a certain amount."""

from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
import numpy as np

import smach
import rospy

class GoForward(smach.State):

    def __init__(self, dist_in_front):
        super(GoForward, self).__init__(outcomes=['Completed_Successfully','Aborted'])

        self.dist_in_front = dist_in_front
        self.position = None

        self.starting_position = None

        self.got_first_odom_msg = False
        self.stopped = False

        rospy.Subscriber('/bump', Bump, self.detect_bump)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.destination_publisher = rospy.Publisher('/destination', PointStamped, queue_size=10)
        # The destination publisher will drop the first message if we don't wait a bit.
        rospy.sleep(1.0)
        rospy.on_shutdown(self.stop)

    def publish_destination(self, x, y, z):
        print 'Publishing destination (%d,%d,%d)' % (x, y, z)
        self.destination_publisher.publish(
            PointStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                point=Point(x, y, z)
            )
        )

    def stop(self):
        self.publisher.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def detect_bump(self, msg):
        left_front_triggered = msg.leftFront
        right_front_triggered = msg.rightFront

        if left_front_triggered or right_front_triggered:
            print('stopping due to bump')
            self.stopped = True
            self.stop()

    def update_odometry(self, msg):
        if self.position is None:
            pos = msg.pose.pose.position
            self.starting_position = np.array([pos.x, pos.y, pos.z])
        current_pos = msg.pose.pose.position
        self.position = np.array([current_pos.x, current_pos.y, current_pos.z])
        if not self.got_first_odom_msg:
            self.got_first_odom_msg = True

    def distance_to(self, point):
        return np.linalg.norm(point - self.position)

    def go_forward(self, distance=1.0):
        r = rospy.Rate(50)
        destination = np.array([distance, 0.0, 0.0])

        move_starting_position = self.position
        self.publish_destination(destination[0], destination[1], destination[2])

        while not rospy.is_shutdown() and self.got_first_odom_msg and not self.stopped:
            distance_from_goal = abs(distance) - self.distance_to(move_starting_position)
            print 'Distance to go: (%f / %f)' % (self.dist_in_front, distance_from_goal)
            if distance_from_goal > 0.001:
                fwd_msg = Twist(linear=Vector3(np.sign(distance)*distance_from_goal, 0.0, 0.0))
                self.publisher.publish(fwd_msg)
            else:
                self.stop()
                return True
            r.sleep()
        return False

    def execute(self, userdata):
        return self.run()

    def run(self):
        r = rospy.Rate(50)
        # Wait for the first odometry position update to come in
        while not self.got_first_odom_msg:
            r.sleep()

        success = self.go_forward(self.dist_in_front)
        if not success:
            return 'Aborted'

        return 'Completed_Successfully'

if __name__ == '__main__':
    rospy.init_node('GoForward')
    GoForward(-1.0).run()
