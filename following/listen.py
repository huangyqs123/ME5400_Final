#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_multiply, quaternion_inverse

amcl_pose = None
dest_pose = None

def amcl_pose_callback(msg):
    rospy.loginfo("Received AMCL pose:")
    global amcl_pose
    amcl_pose = msg.pose

def dest_pose_callback(msg):
    rospy.loginfo("Received dest pose:")
    global dest_pose
    dest_pose = msg.pose

def calculate_middle_point():
    global amcl_pose, dest_pose
    if amcl_pose is None or dest_pose is None:
        return None
    
    # 获取/amcl_pose的位置和方向
    amcl_position = [amcl_pose.pose.position.x, amcl_pose.pose.position.y, amcl_pose.pose.position.z]
    dest_orientation = [dest_pose.pose.orientation.x, dest_pose.pose.orientation.y, dest_pose.pose.orientation.z, dest_pose.pose.orientation.w]
    
    # 获取/dest的位置
    dest_position = [dest_pose.pose.position.x, dest_pose.pose.position.y, dest_pose.pose.position.z]

    # 计算从amcl_position到dest_position的向量
    vector = [dest_position[0] - amcl_position[0], dest_position[1] - amcl_position[1], dest_position[2] - amcl_position[2]]

    # 计算向量的长度
    length = (vector[0]**2 + vector[1]**2 + vector[2]**2)**0.5

    # 单位化向量
    unit_vector = [vector[0]/length, vector[1]/length, vector[2]/length]

    # 计算middle_position
    middle_position = [dest_position[0] - 0.6*unit_vector[0], dest_position[1] - 0.6*unit_vector[1], dest_position[2] - 0.6*unit_vector[2]]

    # 中间点的方向设置为与/amcl_pose相同
    middle_orientation = dest_orientation

    return middle_position, middle_orientation
def move_base_goal_publisher():
    rospy.init_node('move_base_goal_publisher', anonymous=True)
    pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        middle_point = calculate_middle_point()
        if middle_point is not None:
            middle_position, middle_orientation = middle_point
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = middle_position[0]
            goal.pose.position.y = middle_position[1]
            goal.pose.position.z = middle_position[2]
            goal.pose.orientation.x = middle_orientation[0]
            goal.pose.orientation.y = middle_orientation[1]
            goal.pose.orientation.z = middle_orientation[2]
            goal.pose.orientation.w = middle_orientation[3]
            pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber('/tb3_1/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
        rospy.Subscriber('/tb3_0/amcl_pose', PoseWithCovarianceStamped, dest_pose_callback)
        move_base_goal_publisher()
    except rospy.ROSInterruptException:
        pass
