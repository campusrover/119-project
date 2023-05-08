#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
from communication_msg.msg import see_intruder as see_intruder
from communication_msg.msg import stop_order as stop_order
from std_msgs.msg import String


# rip a lot out of it, how ofeten is callback being called, do a rostopic hz, experiment with reducing image size and color, lower the raspicam_node resolution
class Follower:
    def __init__(self,name):
        self.which_robo = name
        self.other_robo_here=False
        self.intruder_msg=see_intruder()
        self.addon=0
        self.move_back=False
        self.ready=False
        self.time_since_lost=0
        self.bridge = cv_bridge.CvBridge()
        self.lower_red=numpy.array([155,25,0])
        self.upper_red=numpy.array([245,222,222])
        # Real robot uses '/raspicam_node/image/compressed'
        self.lower=numpy.array([155,25,0])
        self.higher=numpy.array([245,222,222])
        self.stop = False
        self.ifnextline=None
        self.orderlist=["rob_a","rob_c","rob_b","rob_d"]
        self.dummy=self.which_robo['turtlebot3_core']["tf_prefix"]
        if(self.dummy=="roba"):
            self.order=0
            self.static_order=self.order
        elif(self.dummy=="robb"):
            self.order=2
            self.static_order=self.order
        elif(self.dummy=="robc"):
            self.order=1
            self.static_order=self.order
        elif(self.dummy=="robd"):
            self.order=3
            self.static_order=self.order
        self.which_robo_see = ""
        self.color_dict = {"rob_a": {"color": "red","low": numpy.array([155,25,0]), "high": numpy.array([245,222,222])}, "rob_d": {"color":"green", "low":numpy.array([45, 100, 100]), "high":numpy.array([75, 255, 255])},
        "rob_c": {"color": "blue", "low": numpy.array([90,120,120]),"high":numpy.array([135,220,220])}, "rob_b": {"color": "yellow", "low": numpy.array([20, 100, 100]), "high": numpy.array([45,220,220])}}


        dict = {"name": self.which_robo}
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
        self.detect_intruder_pub = rospy.Publisher('/see_intruder', see_intruder, queue_size=1)
        self.detect_intruder_sub=rospy.Subscriber('/see_intruder', see_intruder,self.see_intruder_callback)
        self.stop_pub = rospy.Publisher('/stop', stop_order, queue_size=1)
        self.stop_sub = rospy.Subscriber('/stop', stop_order, self.stop_callback)
        self.stop_order = stop_order()
        self.stop_order.order_list = []
        self.stop_order.idx.data = -1


# This is the method that will be making the robot find the line that it is going to be following. 
    def see_intruder_callback(self, msg):
        if msg.rob_c.data:
            self.which_robo_see = "rob_c"
        elif msg.rob_b.data:
            self.which_robo_see = "rob_b"
        elif msg.rob_d.data:
            self.which_robo_see = "rob_d"
        
        if(self.which_robo_see!=""):
            if(self.orderlist.index(self.which_robo_see)>self.static_order):
                self.addon=1
            else:
                self.addon=-1
            if(self.ifnextline==True and self.orderlist[self.order]!=self.which_robo_see and self.state=="turning"):
                print("I'm so preppared to change the lineeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee") 
                self.order=self.order+self.addon            
                self.lower=self.color_dict[self.orderlist[self.order]]['low']
                self.higher=self.color_dict[self.orderlist[self.order]]['high']
                self.ifnextline=False
            if(self.state=="finish block"):#When the robot finish blocking
                self.lower=None
                self.higher=None
                self.which_robo_see=None
                self.order=self.static_order
                self.ifnextline=None
        print("myorder"+str(self.order))
        print("This little bitch see the intruder: "+self.which_robo_see)

    def stop_callback(self,msg):
        self.stop_order.order_list = msg.order_list
        if msg.idx.data != -1:
            if msg.order_list[msg.idx.data].data==self.dummy:
                self.move_back = True

# This is the method that will be making the robot find the line that it is going to be following. 
    def image_callback(self, msg):
        mask1=numpy.array([None])
        # get image from camera, real uses compressed_imgmsg_to_cv2
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #if(self.lower.all()==0):
        if(self.which_robo_see==""):
            mask = cv2.inRange(hsv,  self.lower_red, self.upper_red)
            masked = cv2.bitwise_and(image, image, mask=mask)
        else:
            if(self.orderlist[self.order]!=self.which_robo_see):
                print("this is the addon: "+str(self.addon))
                mask1=cv2.inRange(hsv1, self.color_dict[self.orderlist[self.order+self.addon]]['low'],self.color_dict[self.orderlist[self.order+self.addon]]['high'])
            mask = cv2.inRange(hsv,  self.lower, self.higher)
            masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int (3 * h /5)
        search_bot = int (search_top + 20)
        mask[0:search_top, 0:w] = 0
        if(mask1.any()!=None):
            mask1[0:search_top, 0:w] = 0
            print("mask1"+str(cv2.moments(mask1)['m00']))
            if(cv2.moments(mask)['m00']==0 and cv2.moments(mask1)['m00']==0):
                self.ifnextline=False
                print("lose but not need to change the line")
            if(cv2.moments(mask)['m00']==0 and cv2.moments(mask1)['m00']!=0):
                self.ifnextline=True
                print("lose but not need to change the line")
        # mask[search_bot:h, 0:w] = 0
        cv2.imshow("band", mask)
        print("self.ifnextline: "+str(self.ifnextline))

    # Compute the "centroid" and display a red circle to denote it
    
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] == 0 and not self.intruder:
            # if you want it to go move around object then run the line below
            # self.go_to_wall()
            self.state="turning"
            # If you are having it move around object you need to remove the two lines below! 
            # If you want it to move infinitely then you need to have these here. 
            if(self.ifnextline!=False and self.other_robo_here==False and self.which_robo_see!=""):
                self.ifnextline=True
            if(self.other_robo_here==False) and (self.stop == False):
                self.time_since_lost=self.time_since_lost+1
                self.twist.linear.x = 0
                if(self.addon>=0):
                    self.twist.angular.z = 0.5
                else:
                    self.twist.angular.z = -0.5
                self.cmd_vel_pub.publish(self.twist)
            if(self.ifnextline==True and self.move_back==True and self.state=="turning"):
                if(self.static_order>self.orderlist.index(self.which_robo_see)):
                    self.addon=1
                else:
                    self.addon=-1
                self.order=self.order+self.addon
                if(self.order==self.static_order):
                    self.move_back=False  
                    self.stop_order.idx.data-=1
                    self.stop_pub.publish(self.stop_order)          
                self.lower=self.color_dict[self.orderlist[self.order]]['low']
                self.higher=self.color_dict[self.orderlist[self.order]]['high']
                self.ifnextline=False
            if(self.time_since_lost>1500 and self.move_back!=True):
                print("I have waited for sooooo long time! there is no yellow line")
                self.lower=self.lower_red
                self.higher=self.upper_red
                self.order=self.static_order


        if M['m00'] > 0 and not self.intruder and (self.stop == False):
            self.state="following_line"
            cx = int(M['m10']/M['m00']) +100
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            self.time_since_lost=0
            if(self.ifnextline!=None):
                self.ifnextline=None
            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) /1000
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("image", image)
        cv2.waitKey(3)
    
        # Callback function for scan_sub, which will process the scan data and call on function to update states for wall following
    def scan_cb(self, msg):
        ranges = [0 for i in msg.ranges]
        raw_ranges = [i if i>0 else math.inf for i in msg.ranges]
        #raw_ranges = msg.ranges
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
        fronts = front + front_left + front_right
        # get minimum value in each region
        self.region = {0: min(front), 1: min(front_left), 2: min(left), 3: min(back_left), 4: min(back_right), 5: min(right), 6: min(front_right)}
        # update state
        if self.stop==False: 
            self.change_state()
        print(self.which_robo_see + "    " + self.orderlist[self.order]  )
        if(self.orderlist[self.order] == self.which_robo_see): 
            if (min(fronts)<=0.4): 
                if not self.stop:
                    this_robo = String()
                    this_robo.data = self.dummy
                    self.stop_order.order_list.append(this_robo)
                    self.stop_pub.publish(self.stop_order)
        
                self.stop = True
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
   
    # function that will change the state of the robot according to processed scan data
    def change_state(self):
        d = 0
        d1=0.3
        if self.i_state == "finish_turn" and self.region[0]>d and len(self.stop_order.order_list)==2 and not self.ready:
            self.stop_order.idx.data = 1
            print("other robo ready to go back")
            self.stop_pub.publish(self.stop_order)
            self.ready=True
            
        elif self.intruder and self.region[0]<d and ((self.i_state=='turn_left' and self.region[2]>d) or (self.i_state=='turn_right' and self.region[5]>d)):
            self.i_state = "finish_turn"
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        elif self.region[2]<d and self.which_robo_see=="":
            self.intruder = True
            self.i_state = "turn_left"
            msg = see_intruder()
            msg.rob_a.data = True
            print(msg)
            self.detect_intruder_pub.publish(msg)
            self.intruder_msg = msg
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)
            if not self.stop:
                this_robo = String()
                this_robo.data = self.dummy
                self.stop_order.order.append(this_robo)
                self.stop_pub.publish(self.stop_order)
            self.stop=True
            
        elif(self.region[0]<d1 and self.which_robo_see!=""):
            print("other robo is here, stop moving and wait!")
            self.other_robo_here=True
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
        
        elif self.region[5]<d and self.which_robo_see=="":
            self.intruder = True
            self.i_state = "turn_right"
            msg = see_intruder()
            msg.rob_a.data = True
            print(msg)
            self.detect_intruder_pub.publish(msg)
            self.twist.linear.x = 0
            self.twist.angular.z = -0.5
            self.cmd_vel_pub.publish(self.twist)
        elif(self.region[0]>(d1+0.3)):
            self.other_robo_here=False
        self.detect_intruder_pub.publish(self.intruder_msg)
    
    def start(self): 
        return

def main():
    rospy.init_node("Line_roba")
    which_robo = rospy.get_param("~robot_name")
    #name='/'+which_robo['turtlebot3_core']["tf_prefix"]+"follower"
    follower = Follower(which_robo)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        follower.start() 
        rate.sleep()

if __name__ == '__main__':
    main()