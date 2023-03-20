#! /usr/bin/env python
# Karen Mai 
# COSI 119 Autonomous Robotics 
# PA3: Wall Following 

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math
import numpy
from tf.transformations import euler_from_quaternion


class Robot(): 
    def __init__(self,name): 
        
        self.which_robo = name

        # How fast the robot is going to travel and distance away object
        self.angular_speed = 0.2
        self.linear_speed = 0.1
        self.dist_away_object = 0.5

        # Store laser scan values
        self.scanner = []
        self.ranges = {
            'right': 0, 
            'back': 0, 
            'left': 0, 
            'front': 0
        }

        # Updating states
        self.state = {
            'find_object': True,
            'go_object': False,
            'realign_against_object': False,
            'running': False, 
            'running_continues': False
        }

        # Potential needed fields
        self.angle_turn = {
            'front': 0, 
            'right': 90, 
            'back': 180, 
            'left': 270
        }
        self.position = { 
            'x': 0, 
            'y': 0, 
            'z': 0
        }
       
        self.loc_wall = ""

        # Simple set ups 
        self.yaw = 0
        scan_sub = rospy.Subscriber(f'/{self.which_robo}/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber (f'/{self.which_robo}/odom', Odometry, self.odom_cb)
        self.vel_pub = rospy.Publisher(f'/{self.which_robo}/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

         # used for PID 
        self.threshold_degrees = 20
        self.kP = 1.57
        self.roll = self.pitch = self.yaw = 0.0
        self.realign_rotation = 90
        self.right_point = 0
        self.direction = -1 
        self.p = 15 
        self.d = 0 
        self.angle = 1 
        self.direction = -1
        self.e = 0 
        self.min_angle = 0 
        self.front_dist = 0 
        self.diff_e = 0 # difference between current error and previous error 
        self.min_dist = 0
        self.views = []
        self.object = 'right'
        self.startTime = 0

    # Performs data clean and motions when following object based on lasers
    def scan_cb(self, msg): 
        self.views = msg.ranges
        self.right_point = msg.ranges[90]
        right_raw = list(msg.ranges[45:134])
        right_edited = [right_raw[x] for x in range(len(right_raw))if (math.inf != right_raw[x])] 
        if (len(right_edited) > self.threshold_degrees): 
            self.ranges['right'] = (sum(right_edited))/len(right_edited)
        else: 
            self.ranges['right'] = (sum(right_raw))/90

        back_raw = list(msg.ranges[135:224])
        back_edited = [back_raw[x] for x in range(len(back_raw)) if (math.inf != back_raw[x])] 
        # print(back_edited)
        if (len(back_edited) > self.threshold_degrees): 
            self.ranges['back'] = (sum(back_edited))/len(back_edited)
        else: 
            self.ranges['back'] = (sum(back_raw))/90

        left_raw = list(msg.ranges[225:314])
        left_edited = [left_raw[x] for x in range(len(left_raw)) if (math.inf != left_raw[x])] 
        if (len(left_edited) > self.threshold_degrees): 
            self.ranges['left'] = (sum(left_edited))/len(left_edited)
        else: 
            self.ranges['left'] = (sum(left_raw))/90  
              
        front_raw = numpy.append(msg.ranges[315:359],(msg.ranges[0:44]))
        front_edited = [front_raw[x] for x in range(len(front_raw)) if (math.inf != front_raw[x])] 
        # print(front_edited)
        if (len(front_edited) > self.threshold_degrees): 
            self.ranges['front'] = (sum(front_edited))/len(front_edited)
        else: 
            self.ranges['front'] = (sum(front_raw))/90
        
        # Reset object distance, should also edit within field
        distance_wall = 0.5
        Kd = 0 
        angle = 90
        Kp = 1.0
        Kp2 = 1.0 

        # Performs all of the object following: came from the paper on Gradescope 
        if self.state["running_continues"] == True: 
            laser = numpy.array(msg.ranges) # Uses the laser scan data
            laser[laser<msg.range_min] = 999
            laser[laser==float('inf')] = 999
            Dmin_idx, Dmin = min(enumerate(laser), key=lambda x: x[1]) # Using the min
            angle = msg.angle_min + Dmin_idx * (msg.angle_max - msg.angle_min)/len(laser)  # Mentioned in paper
            t = Twist() 
            t.linear.x = self.linear_speed
            Error = Dmin - distance_wall # Calculating Error
            Wpd = (Kp * Error + Kd * (Error - ErrorPrevious) / 0.1) if Dmin < 0.1 else 0 # Formula from Paper
            direction = -1 if laser[269] < laser[89] else 1 # Checking which side to be following 
            Wp = Kp2 * (angle-math.pi/2*direction) # This now the Wp to be added to Wpd for angular z
            t.angular.z = Wpd + Wp 
            ErrorPrevious = Error 
            self.vel_pub.publish(t) 
        
    # Finding the object   
    def find_object(self): 
        self.state['find_object'] = False
        ranges_here = self.ranges
        shortest_dist = min(ranges_here.values())
        location = [key for key in ranges_here if ranges_here[key] == shortest_dist]
        self.state['find_object'] = True
        print(self.ranges)
        print("Location", location)
        return (location[0])

    # Going up to the object and update states
    def around_object(self): 
        if(self.state['find_object'] == True):
            self.loc_wall = self.find_object()
            self.turn()
        if (self.state['go_object'] == True): 
            self.drive_to_object()
        if(self.state['realign_against_object'] == True):
            self.realign_to_object()
        if(self.state['running'] == True): 
            self.startTime = rospy.Time.now() 
            self.state['running'] = False
            self.state['running_continues'] = True
            print("Starting up Follow")
        if(self.state['running_continues'] == True): 
            print("Following Object")

    # Making robot turn proper degrees
    def turn(self): 
        """ Turn till robot faces front """
        print(self.loc_wall)
        goal_distance = self.ranges[self.loc_wall] # distance of how far front robot should be 
        print(self.loc_wall)
        relative_angle = self.angle_turn[self.loc_wall]*2*(math.pi)/360
        self.vel_msg.angular.z = self.kP  * (relative_angle - self.yaw)
        self.vel_pub.publish(self.vel_msg)
        if(self.vel_msg.angular.z < 0.01): 
            self.state['find_object'] = False
            self.state['go_object'] = True
    
    # Driving up to the object
    def drive_to_object(self): 
        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z = 0
        while(self.ranges['front'] > self.dist_away_object): 
            self.vel_pub.publish(self.vel_msg)
        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)
        print(self.ranges['front'])
        if(self.ranges['front'] <= self.dist_away_object): 
            print("I am done")
            self.state['go_object'] = False
            self.state['realign_against_object'] = True

    # Realign so that it is parallel to object
    def realign_to_object(self): 
        relative_angle = self.realign_rotation*2*(math.pi)/360
        self.vel_msg.angular.z = self.kP  * (relative_angle - self.yaw)
        self.vel_pub.publish(self.vel_msg)
        print(self.vel_msg.angular.z)
        if(abs(self.vel_msg.angular.z) < 0.01): 
            self.state['realign_against_object'] = False
            self.state['running'] = True

    # Added in case need method
    def odom_cb(self,msg): 
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        self.position['z'] = msg.pose.pose.position.z

# Main Function to run 
def main():
    rospy.init_node('wall_follower')
    which_robo = rospy.get_param("~robot_name")
    wall_robot = Robot(which_robo)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wall_robot.around_object() 
        rate.sleep()

if __name__ == '__main__':
    main()