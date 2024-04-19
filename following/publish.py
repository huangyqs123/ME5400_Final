#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/dest', PoseStamped, queue_size=1)
    
    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)
    
    mypose=PoseStamped()
    mypose.header.frame_id='map' #设置自己的目标
    mypose.pose.position.x=-0.14658700327138624
    mypose.pose.position.y=-4.746676775769442
    mypose.pose.position.z=0
    mypose.pose.orientation.x=0
    mypose.pose.orientation.y=0
    mypose.pose.orientation.z=-0.009900646643846149
    mypose.pose.orientation.w=0.9999509873968992

    update_count = 0
    while update_count < 22:  # 只更新25次
        mypose.pose.position.x += 0.3
        mypose.pose.position.y += 0.15
        turtle_vel_pub.publish(mypose) #发送自己设置的目标点
        time.sleep(2)  # 暂停一秒
        update_count += 1  # 更新计数器



