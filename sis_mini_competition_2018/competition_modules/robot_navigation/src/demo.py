#!/usr/bin/env python
import rospy
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseStamped
from robot_navigation.srv import robot_navigation
from math import sqrt
import time

class TagTracker(object):
    def __init__(self):
        # the tag to track
        self.tag_id_track = 1

        # the distance threshold between car and tag
        self.thres_distance = 0.5

        # the angle threshold between car and tag
        self.thres_angle = 0.1

        # the flag for whether the robot has arrived the destination
        self.done_tracking = False

        # the lock for cbTag in case that calling service again before it returns
        self.cb_lock = False

        # setup service proxy
        rospy.wait_for_service('/robot_navigate')
        

        # setup subsciber
        self.sub_tag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cbTag, queue_size = 1)

        

    def cbTag(self, msg):
        if self.cb_lock == True:
            print "lock == true"
            return
        if len(msg.detections) == 0:
            print "no detection of id %d"%{tag_id_track}
            return
        if self.done_tracking == True:
            print "done tracking"
            return
        self.cb_lock = True
        rospy.loginfo("cbTag")
        for detection in msg.detections:
            if detection.id[0] == self.tag_id_track:
                rospy.loginfo("Tag %d found, start tracking..." %(self.tag_id_track))
                print "x: " + str(detection.pose.pose.pose.position.x)
                print "z: " + str(detection.pose.pose.pose.position.z)
                distance = sqrt(detection.pose.pose.pose.position.x ** 2 + detection.pose.pose.pose.position.z ** 2)
                print "distance: " + str(distance)
                if distance <= self.thres_distance:
                    rospy.loginfo("The robot has arrived its destination")
                    exit(1)
                else:
                    try:
                        rospy.wait_for_service('/robot_navigate')
                        ser_tag = rospy.ServiceProxy('/robot_navigate', robot_navigation)
                        print (ser_tag(detection.id[0]))
                        time.sleep(3)
                        self.cb_lock = False

                    except rospy.ServiceException as e:
                        print e
                        exit(1)

if __name__ == "__main__":
    rospy.init_node("tag_tracker",anonymous=False)
    joy_mapper = TagTracker()
    rospy.spin()
