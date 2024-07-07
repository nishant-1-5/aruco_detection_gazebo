#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(gray)

        if markerIds is not None:
            print(f"Detected markers: {markerIds.flatten()}")
            cv_image = cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

        cv.imshow("Aruco Markers", cv_image)
        cv.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

