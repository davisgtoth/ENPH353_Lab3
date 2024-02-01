#! /usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower():
    def __init__(self):
        rospy.init_node('line_follower')
        
        self.bridge = CvBridge()
        
        self.lin_speed = 0.5
        self.rot_speed = 4.0

        self.kp = 10
        self.kd = 2
        self.prevError = 0
        self.left = True
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber("/robot/camera/image_raw", Image, self.callback)

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        error = self.calcError(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        p = self.kp * error
        d = self.kd * (error - self.prevError)
        self.drive(p + d)
        self.prevError = error

    def calcError(self, frame, bin_thresh=100, buffer=20):
        height = frame.shape[0]
        width = frame.shape[1]
        ret, bin_frame = cv2.threshold(frame, bin_thresh, 255, cv2.THRESH_BINARY)

        # cv2.imshow('window', bin_frame)
        # cv2.waitKey(0)

        Ylvl = height - buffer 

        leftIndex = -1
        rightIndex = -1

        for i in range(width):
            if bin_frame[Ylvl][i] == 0 and leftIndex == -1:
                leftIndex = i
            elif bin_frame[Ylvl][i] == 0 and leftIndex != -1:
                rightIndex = i       
        
        roadCentre = -1
        if leftIndex != -1 and rightIndex != -1:
            roadCentre = (rightIndex + leftIndex) // 2
        elif rightIndex == -1:
            roadCentre = leftIndex // 2

        if roadCentre != -1:
            error = (width // 2) - roadCentre
            self.left = error > 0
        else:
            error = ((self.left * 2) - 1) * (width / 2)
    
        return error / width
        
    def drive(self, error):
        move = Twist()
        move.linear.x = self.lin_speed
        move.angular.z = self.rot_speed * error
        self.vel_pub.publish(move)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        my_line_follower = LineFollower()
        my_line_follower.start()
    except rospy.ROSInterruptException:
        pass    