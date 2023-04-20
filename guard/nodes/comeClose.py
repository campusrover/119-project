#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion

class Close:
    def __init__(self,name):
        self.which_robo = name
        dict = {"name": self.which_robo}
        self.dummy = '/'+self.which_robo['turtlebot3_core']["tf_prefix"]
        self.dummy2 = self.which_robo['turtlebot3_core']["tf_prefix"]

        print(self.dummy)
        self.cmd_vel_pub = rospy.Publisher(self.dummy+'/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.state = 'coming'
        self.turned = False
        self.scan_sub = rospy.Subscriber(f'/{self.dummy2}/scan', LaserScan, self.scan_cb)
        self.vel_msg = Twist()
        print(f'/{self.dummy2}/scan')
    
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
        self.vel_msg.linear.x = 0.5 #Forcing our robot to stop
        print("here")
        self.cmd_vel_pub.publish(self.vel_msg) 
        if(min(front) < 0.5):
            turn(self,90,0.5)
    
    def rotateRightHere(self,angle,speed):
        #Converting from angles to radians
        angular_speed = speed*2*PI/360; relative_angle = angle*2*PI/360
        self.vel_msg.angular.z = abs(angular_speed) # Rotating speed
        t0 = rospy.Time.now().to_sec() # Time for distance calculation
        current_angle = 0 # Currently at 0 degrees 
        while(current_angle < relative_angle):
            print("Rotating at: " + str(current_angle))
            self.cmd_vel_pub.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
        self.vel_msg.angular.z = 0 #Forcing our robot to stop
        self.cmd_vel_pub.publish(self.vel_msg) 
    def start(self): 
        self.vel_msg.linear.x = 0.5 #Forcing our robot to stop
        print("here")
        self.cmd_vel_pub.publish(self.vel_msg) 

def main():
    rospy.init_node("closingUp")
    which_robo = rospy.get_param("~robot_name")
    #name='/'+which_robo['turtlebot3_core']["tf_prefix"]+"follower"
    follower = Close(which_robo)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        follower.start()
        rate.sleep()

if __name__ == '__main__':
    main()