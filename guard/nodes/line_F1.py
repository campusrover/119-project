#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
from communication_msg.msg import see_intruder as see_intruder


# rip a lot out of it, how ofeten is callback being called, do a rostopic hz, experiment with reducing image size and color, lower the raspicam_node resolution
class Follower:
    def __init__(self,name):
        self.which_robo = name

        self.bridge = cv_bridge.CvBridge()
        # Real robot uses '/raspicam_node/image/compressed'


        dict = {"name": self.which_robo}
        print(self.which_robo['turtlebot3_core']["tf_prefix"])
        # print(self.which_robo)

        self.dummy = '/'+self.which_robo['turtlebot3_core']["tf_prefix"]

        self.image_sub = rospy.Subscriber(self.dummy+'/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher(self.dummy+'/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

                # state for line following
        self.state = 'find'
        # states for intruder detection
        self.intruder = False
        self.i_state = ""
        self.region = {}
        self.scan_sub = rospy.Subscriber(self.dummy+'/scan', LaserScan, self.scan_cb)
        self.vel_msg = Twist()
        self.detect_intruder_pub = rospy.Publisher(self.dummy+'/see_intruder', see_intruder, queue_size=1)
        self.detect_intruder_sub=rospy.Subscriber(self.dummy+'/see_intruder', see_intruder,self.see_intruder_callback)
        self.which_robo_see = ""
        self.color_dict = {"rob_a": {"color": "red","low": numpy.array([155,25,0]), "high": numpy.array([245,222,222])}, "rob_b": {"color":"green", "low":numpy.array([45, 100, 100]), "high":numpy.array([75, 255, 255])},
        "rob_c": {"color": "blue", "low": numpy.array([110,50,50]),"high":numpy.array([130,255,255])}, "rob_d": {"color": "yellow", "low": numpy.array([5, 100, 100]), "high": numpy.array([40, 255, 255])}}


# This is the method that will be making the robot find the line that it is going to be following. 
    def see_intruder_callback(self, msg):
        if msg.rob_b.data:
            self.which_robo_see = "rob_b"
        elif msg.rob_c.data:
            self.which_robo_see = "rob_c"
        elif msg.rob_d.data:
            self.which_robo_see = "rob_d"



# This is the method that will be making the robot find the line that it is going to be following. 
    def image_callback(self, msg):

        # get image from camera, real uses compressed_imgmsg_to_cv2
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([155,25,0])  # [ 40, 0, 0], ([5, 100, 100]) 
        upper_red = numpy.array([245,222,222 ]) # [ 120, 255, 255], [40, 255, 255]
        # what works for the in person tape is [5, 100, 100] - [ 100, 210, 210]
        # print(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        mask = cv2.inRange(hsv,  lower_red, upper_red)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int (3 * h /4)
        search_bot = int (search_top + 20)
        mask[0:search_top, 0:w] = 0
        # mask[search_bot:h, 0:w] = 0
        cv2.imshow("band", mask)

    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] == 0 and not self.intruder:
            # if you want it to go move around object then run the line below
            # self.go_to_wall()

            # If you are having it move around object you need to remove the two lines below! 
            # If you want it to move infinitely then you need to have these here. 
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)

        if M['m00'] > 0 and not self.intruder:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) /5000
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
        if self.intruder and self.region[0]<d and ((self.i_state=='turn_left' and self.region[2]>d) or (self.i_state=='turn_right' and self.region[5]>d)):
            self.i_state = "finish_turn"
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        elif self.region[2]<d:
            self.intruder = True
            self.i_state = "turn_left"
            msg = see_intruder()
            msg.rob_b.data = True
            print(msg)
            self.detect_intruder_pub.publish(msg)
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)
        
        elif self.region[5]<d:
            self.intruder = True
            self.i_state = "turn_right"
            msg = see_intruder()
            msg.rob_b.data = True
            print(msg)
            self.detect_intruder_pub.publish(msg)
            self.twist.linear.x = 0
            self.twist.angular.z = -0.5
            self.cmd_vel_pub.publish(self.twist)
    
    
    def start(self): 
        print("I am running")

def main():
    rospy.init_node("LineF")
    which_robo = rospy.get_param("~robot_name")
    #name='/'+which_robo['turtlebot3_core']["tf_prefix"]+"follower"
    follower = Follower(which_robo)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        follower.start() 
        rate.sleep()

if __name__ == '__main__':
    main()
