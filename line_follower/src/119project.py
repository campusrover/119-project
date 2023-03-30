#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # subscribe to /camera/rgb/image_raw to see the line
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # subscribe to scan to detect obstacles
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        # create publisher to make the robot move
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0
        # state for line following
        self.state = 'find'
        # states for intruder detection
        self.intruder = False
        self.i_state = ""
        self.region = {}

    # Callback function for image_sub, which will process image recieved, update states of the robot, and update cmd_vel according to states of the robot
    def image_callback(self, msg):
        print("I am doing a cb on the iamge")
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 40, 0, 0])
        upper_yellow = numpy.array([ 120, 255, 255])
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        cv2.imshow("band", mask)

    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        # The robot is currently doing wall following 
        if self.intruder:
            if self.i_state == 'turn':
                self.twist.linear.x = 0
                self.twist.angular.z = 0.2
                self.cmd_vel_pub.publish(self.twist)
            if self.i_state == 'finish_turn':
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
        else:
        
            # The robot sees a centroid to follow
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00']) + 100
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                # Move at 0.2 M/sec
                # add a turn if the centroid is not in the center
                # Hope for the best. Lots of failure modes.
                err = cx - w/2
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 1000
                self.cmd_vel_pub.publish(self.twist)

            # There's no centroid in robot's view
            else:
                # follows to the end: needs to turn and follow line back
                if self.state == 'follow':
                    print("turn")
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0.2
                    self.cmd_vel_pub.publish(self.twist)
                # not finding a line: make circular move to find the line
                elif self.state == 'find':
                    print("find")
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -0.2
                    self.cmd_vel_pub.publish(self.twist)
                    
        cv2.imshow("image", image)
        cv2.waitKey(3)

    # Callback function for scan_sub, which will process the scan data and call on function to update states for wall following
    def scan_cb(self, msg):
        ranges = [0 for i in msg.ranges]
        #raw_ranges = [i if i>0 else math.inf for i in msg.ranges]
        raw_ranges = msg.ranges
        # make each data the average of itself and four nearby data to avoid noise
        for i in range(len(raw_ranges)):
            ranges[i] = (raw_ranges[i-2]+raw_ranges[i-1]+raw_ranges[i]+raw_ranges[(i+1)%(len(raw_ranges))]+raw_ranges[(i+2)%(len(raw_ranges))])/5
        # split the full ranges into regions
        front = ranges[0:17]+ranges[342:]
        front_left = ranges[18:53]
        left = ranges[54:89]
        back_left = ranges[90:179]
        back_right = ranges[180:269]
        right = ranges[270:305] 
        front_right = ranges[306:341]
        # get minimum value in each region
        self.region = {0: min(front), 1: min(front_left), 2: min(left), 3: min(back_left), 4: min(back_right), 5: min(right), 6: min(front_right)}
        # update state
        self.change_state()

    # function that will change the state of the robot according to processed scan data
    def change_state(self):
        d = 0.5
        if self.intruder and self.region[0]<d:
            self.i_state = "finish_turn"

        elif self.region[2]<d:
            self.intruder = True
            self.i_state = "turn"

        
print("Hello")
# Initialize the node and let the node spin
rospy.init_node('follower')
follower = Follower()
rospy.spin()