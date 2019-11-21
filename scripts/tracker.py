#!/usr/bin/env python
# coding: UTF-8

import cv2
import numpy as np
import rospy
import message_filters
import cv_bridge
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image


class Tracking():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.algorithm = rospy.get_param("~algorithm", "kcf")
        self.create_tracker()
        self.INPUT_IMAGE = rospy.get_param(
            "~input_image",
            "/head_mount_kinect/hd/image_color_rect_repub_desktop")
        self.INPUT_IMAGE = rospy.get_param(
            "~input_image",
            "/apply_mask_image_in_gripper/output")
        self.BOX = rospy.get_param(
            "~box",
            "/mask_rcnn_instance_segmentation/output/rects")
        self.missing_box = True
        queue_size = rospy.get_param("~queue_size", 1)
        self.image_pub = rospy.Publisher(
            "~output", Image, queue_size=queue_size)
        sub_image = message_filters.Subscriber(
            self.INPUT_IMAGE, Image, queue_size=queue_size)
        sub_box = message_filters.Subscriber(
            self.BOX, RectArray, queue_size=10)
        self.subs = [sub_image, sub_box]
        if rospy.get_param("~approximate_sync", True):
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def create_tracker(self):
        rospy.loginfo("create tracker")
        if self.algorithm == "kcf":
            self.tracker = cv2.TrackerKCF_create()
        elif self.algorithm == "tld":
            self.tracker = cv2.TrackerTLD_create()
        elif self.algorithm == "goturn":
            self.tracker = cv2.TrackerGOTURN_create()
        else:
            rospy.logerr(rospy.get_param("algorithm") + "is not implemented")
            return

    def put_text(self, image, text, pt):
        cv2.putText(
            image, text, pt, cv2.FONT_HERSHEY_SIMPLEX, 3,
            (0, 255, 0), 3, cv2.LINE_AA)

    def callback(self, imgmsg, boxmsg):

        image = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding="bgr8")
        if(self.missing_box and len(boxmsg.rects) != 0):
            rospy.loginfo("tracker init")
            init_box = (boxmsg.rects[0].x - 10,
                        boxmsg.rects[0].y - 10,
                        boxmsg.rects[0].width + 20,
                        boxmsg.rects[0].height + 20)
            self.tracker.init(image, init_box)
            self.missing_box = False

        track, bbox = self.tracker.update(image)
        if(track):
            cv2.rectangle(image,
                          (int(bbox[0]), int(bbox[1])),
                          (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])),
                          (0, 255, 0), 5, 1)
            rospy.loginfo("tracking")
            self.put_text(image, "Tracking", (10, 300))
            self.missing_box = False
        else:
            rospy.loginfo("missing")
            self.put_text(image, "Missing", (10, 300))
            self.missing_box = True

        self.put_text(image, self.algorithm, (10, 150))
        vis_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        vis_msg.header = imgmsg.header
        self.image_pub.publish(vis_msg)


if __name__ == '__main__':
    rospy.init_node("tracking", anonymous=False)
    tracking = Tracking()
    rospy.spin()
