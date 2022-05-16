#!/usr/bin/env/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String 


# key_mapping is [angular, linear]

key_mapping = {'w': [0, 1], 'x': [0, -1], 'a': [1, 0],
            'd': [-1, 0], 's': [0, 0]}


g_vel_scales = [0.1, 0.1]
max_speeds = [1, 1]

g_add_twist = Twist() 
last_twist = Twist()  
twist_to_pub = Twist()         
speed_increment = [0, 0]
prev_speed = [0, 0]
cmd_speed = [0, 0]

def keys_cb(msg):
    global g_add_twist, twist_to_pub
    if len(msg.data) == 0 or msg.data not in key_mapping.keys():
        return
    twist_to_pub = Twist()
    if msg.data == 's':
        twist_to_pub.linear.x = 0
        twist_to_pub.angular.z = 0
    else:
        vels = key_mapping[msg.data]

        speed_increment[0] = vels[0] * g_vel_scales[0]
        speed_increment[1] = vels[1] * g_vel_scales[1]
        
        prev_speed[0] = last_twist.angular.z
        prev_speed[1] = last_twist.linear.x
        

        cmd_speed[0] = prev_speed[0] + speed_increment[0]
        cmd_speed[1] = prev_speed[1] + speed_increment[1]
        
        cmd_speed_clipped = check_limit(cmd_speed, max_speeds)
        twist_to_pub.linear.x = round(cmd_speed_clipped[1], 2)
        twist_to_pub.angular.z = round(cmd_speed_clipped[0], 2)

def check_limit(current_speed, max_speed):
    #current_speed = [angular_speed, linear_speed]
    #max_speed = [MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED]

    if current_speed[0] > max_speed[0]:
        current_speed[0] = max_speed[0]
    if current_speed[1] > max_speed[1]:
        current_speed[1] = max_speed[1]
    if current_speed[0] < -max_speed[0]:
        current_speed[0] = -max_speed[0]
    if current_speed[1] < -max_speed[1]:
        current_speed[1] = -max_speed[1]
    return current_speed
   
def get_last_twist(msg):
    global last_twist
    last_twist = msg
    
if __name__ == '__main__':

    rospy.init_node('keys_to_twist_with_ramps', anonymous=True)

    if rospy.has_param('linear_scale'):
        g_vel_scales[1] = rospy.get_param('linear_scale')
    else:
        rospy.logwarn(f'linear scale not set; using {g_vel_scales[1]} instead')

    if rospy.has_param('angular_scale'):
        g_vel_scales[0] = rospy.get_param('angular_scale')
    else:
        rospy.logwarn(f'angular scale not set; using {g_vel_scales[0]} instead')

    if rospy.has_param('max_linear_speed'):
        max_speeds[1] = rospy.get_param('max_linear_speed')
    else: 
        rospy.logwarn(f'max linear speed limit is not set; using {max_speeds[1]} instead')
    
    if rospy.has_param('max_angular_speed'):
        max_speeds[0] = rospy.get_param('max_angular_speed')
    else:
        rospy.logwarn(f'max angular speed limit is not set; using {max_speeds[0]} instead')

    twist_sub = rospy.Subscriber('cmd_vel', Twist, get_last_twist)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    key_sub = rospy.Subscriber('keys', String, keys_cb)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        twist_pub.publish(twist_to_pub)
        rate.sleep()

