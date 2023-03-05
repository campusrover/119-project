#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
# Create a node and allocate a tf buffer and a tf listener.
def intruder_distance(trans):
    distance=math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
    return distance
def def_new_trans(face_intruder,guardw,guards,guarde,guardn):
    return [tfBuffer.lookup_transform(guardw, face_intruder, rospy.Time()),tfBuffer.lookup_transform(guards, face_intruder, rospy.Time()),tfBuffer.lookup_transform(guarde, face_intruder, rospy.Time()),tfBuffer.lookup_transform(guardn, face_intruder, rospy.Time())]
def find_move_path(face_intruder,trans):
    if(face_intruder.find("w")>0):
        w_move=[0,0]
        s_move=[4 *math.atan2(trans[1].transform.translation.y+1, trans[1].transform.translation.x),0.5 * math.sqrt(trans[1].transform.translation.x ** 2 + (trans[1].transform.translation.y+1) ** 2)]
        e_move=[4 *math.atan2(trans[2].transform.translation.y-2, trans[2].transform.translation.x),0.5 * math.sqrt(trans[2].transform.translation.x ** 2 + (trans[2].transform.translation.y-2) ** 2)]
        n_move=[4 *math.atan2(trans[3].transform.translation.y-1, trans[3].transform.translation.x),0.5 * math.sqrt(trans[3].transform.translation.x ** 2 + (trans[3].transform.translation.y-1) ** 2)]
    elif(face_intruder.find("e")>0):
        w_move=[4 *math.atan2(trans[0].transform.translation.y+1, trans[0].transform.translation.x),0.5 * math.sqrt(trans[0].transform.translation.x ** 2 + (trans[0].transform.translation.y+1) ** 2)]
        s_move=[4 *math.atan2(trans[1].transform.translation.y-2, trans[1].transform.translation.x),0.5 * math.sqrt(trans[1].transform.translation.x ** 2 + (trans[1].transform.translation.y+1) ** 2)]
        e_move=[0,0]
        n_move=[4 *math.atan2(trans[3].transform.translation.y-1, trans[3].transform.translation.x),0.5 * math.sqrt(trans[3].transform.translation.x ** 2 + (trans[3].transform.translation.y-1) ** 2)]
    elif(face_intruder.find("n")>0):
        w_move=[4 *math.atan2(trans[0].transform.translation.y, trans[0].transform.translation.x+2),0.5 * math.sqrt((trans[0].transform.translation.x+2) ** 2 + (trans[0].transform.translation.y) ** 2)]
        s_move=[4 *math.atan2(trans[1].transform.translation.y, trans[1].transform.translation.x-1),0.5 * math.sqrt((trans[1].transform.translation.x-1) ** 2 + (trans[1].transform.translation.y) ** 2)]
        e_move=[4 *math.atan2(trans[2].transform.translation.y, trans[2].transform.translation.x+1),0.5 * math.sqrt((trans[2].transform.translation.x-1) ** 2 + (trans[2].transform.translation.y) ** 2)]
        n_move=[0,0]
    elif(face_intruder.find("s")>0):
        w_move=[4 *math.atan2(trans[0].transform.translation.y, trans[0].transform.translation.x+1),0.5 * math.sqrt((trans[0].transform.translation.x+1) ** 2 + (trans[0].transform.translation.y) ** 2)]
        s_move=[0,0]
        e_move=[4 *math.atan2(trans[2].transform.translation.y, trans[2].transform.translation.x+1),0.5 * math.sqrt((trans[2].transform.translation.x-1) ** 2 + (trans[2].transform.translation.y) ** 2)]
        n_move=[4 *math.atan2(trans[3].transform.translation.y, trans[3].transform.translation.x-1),0.5 * math.sqrt((trans[3].transform.translation.x-1) ** 2 + (trans[3].transform.translation.y) ** 2)]
    return [w_move,s_move,e_move,n_move]
if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    


# Advanced trick to launch a second copy of the turtlesim named after
# the param turtle (default to turtle2)
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    guard_w = rospy.get_param('turtle', 'guardw')
    guard_s = rospy.get_param('turtle', 'guards')
    guard_e = rospy.get_param('turtle', 'guarde')
    guard_n = rospy.get_param('turtle', 'guardn')
    intruder = rospy.get_param('turtle', 'intruder')
    """
    Add a turtle and also spawn it in case we need three turtle
    """
    turtle_name2 = rospy.get_param('turtle', 'turtle3')
    spawner(4-0.5, 6-0.5, 0, guard_w)
    spawner(6-0.5, 4-0.5, 0, guard_s)
    spawner(8-0.5, 6-0.5, 0, guard_e)
    spawner(6-0.5, 8-0.5, 0, guard_n)
    spawner(6-0.5, 4-1, 0, intruder)



# We are going to steer turtle2. We start by creating a turtle2/cmd_vel publisher
    w_vel = rospy.Publisher('%s/cmd_vel' % guard_w, geometry_msgs.msg.Twist, queue_size=1)
    s_vel = rospy.Publisher('%s/cmd_vel' % guard_s, geometry_msgs.msg.Twist, queue_size=1)
    e_vel = rospy.Publisher('%s/cmd_vel' % guard_e, geometry_msgs.msg.Twist, queue_size=1)
    n_vel = rospy.Publisher('%s/cmd_vel' % guard_n, geometry_msgs.msg.Twist, queue_size=1)


# And we loop, 10x per second
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
# This is the most important line. Requests the transform between turtle1 and turtle_name
            trans_w = tfBuffer.lookup_transform(guard_w, 'intruder', rospy.Time())
            trans_s = tfBuffer.lookup_transform(guard_s, 'intruder', rospy.Time())
            trans_e = tfBuffer.lookup_transform(guard_e, 'intruder', rospy.Time())
            trans_n = tfBuffer.lookup_transform(guard_n, 'intruder', rospy.Time())
            trans_i = tfBuffer.lookup_transform(guard_n, 'intruder', rospy.Time())
            distance_list=[intruder_distance(trans_w),intruder_distance(trans_s),intruder_distance(trans_e),intruder_distance(trans_n)]
            guard_list=["guardw","guards","guarde","guardn"]
            min_dis=distance_list.index(min(distance_list))
            face_intruder=guard_list[min_dis]
            trans_protect=def_new_trans(face_intruder,guard_w,guard_s,guard_e,guard_n)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg_w = geometry_msgs.msg.Twist()
        msg_s = geometry_msgs.msg.Twist()
        msg_e = geometry_msgs.msg.Twist()
        msg_n = geometry_msgs.msg.Twist()

# Some trig to compute the desired motion of turtle2
        movement=find_move_path(face_intruder,trans_protect)
        msg_w.angular.z = movement[0][1]
        msg_w.linear.x = movement[0][0]
        msg_s.angular.z = movement[1][1]
        msg_s.linear.x = movement[1][0]
        msg_e.angular.z = movement[2][1]
        msg_e.linear.x = movement[2][0]
        msg_n.angular.z = movement[3][1]
        msg_n.linear.x = movement[3][0]

# And publish it to drive turtle2
        if(face_intruder.find("e")>0 or face_intruder.find("w")>0):
            thresholdw=trans_protect[0].transform.translation.y
            thresholds=trans_protect[1].transform.translation.y
            thresholde=trans_protect[2].transform.translation.y
            thresholdn=trans_protect[3].transform.translation.y
        if(face_intruder.find("s")>0 or face_intruder.find("n")>0):
            thresholdw=trans_protect[0].transform.translation.x
            thresholds=trans_protect[1].transform.translation.x
            thresholde=trans_protect[2].transform.translation.x
            thresholdn=trans_protect[3].transform.translation.x
        print(math.sqrt(thresholdw**2))
        
        #if(math.sqrt(thresholdw**2)>0.1 or math.sqrt(thresholdw**2)<-0.1):
        w_vel.publish(msg_w)
        """
        else:
            print("in")
            msg_w.linear.x=0
            msg_w.angular.z=0
            w_vel.publish(msg_w)

        if(math.sqrt(thresholds**2)>0.1 or math.sqrt(thresholds)>0.1):
            s_vel.publish(msg_s)
        if(math.sqrt(thresholde**2)>0.5):
            e_vel.publish(msg_e)
        if(math.sqrt(thresholdn**2)>0.5):
            n_vel.publish(msg_n)
        if(math.sqrt(thresholde**2)>0.5):
            e_vel.publish(msg_e)
        """
        rate.sleep()
