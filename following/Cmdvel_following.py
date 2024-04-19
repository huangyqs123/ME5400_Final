import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class TrackerStatusHandler(object):

    def __init__(self):
        self._sub_status = rospy.Subscriber('/tracker/status', Bool, self._process_status, queue_size=1)
        self._sub_offset = rospy.Subscriber('/tracker/offset', Float32, self._process_offset, queue_size=1)
        self._pub_cmd_vel = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self._sub_distance = rospy.Subscriber('/tracker/distance', Float32, self._process_distance, queue_size=1)
        self._offset = 0.0

    def _process_status(self, msg):
        if msg.data:  # 如果状态为True
            # 创建一个Twist消息，设置线速度和角速度
            twist = Twist()
            twist.angular.z = 0.2  # 设置一个适当的旋转速度
            self._pub_cmd_vel.publish(twist)  # 发布消息

        else:  # 如果状态为False
            # 根据offset的值来决定旋转的方向
            rospy.sleep(1)
            twist = Twist()
            if -0.08 <= self._offset <= 0.08:
                twist.linear.z = 0
            elif self._offset > 0.1:
                twist.angular.z = -0.15
            elif self._offset < -0.1:
                twist.angular.z = 0.15
            self._pub_cmd_vel.publish(twist)  # 发布消息


    def _process_offset(self, msg):
        self._offset = msg.data  # 更新offset的值‘

    def _process_distance(self, msg):
        if msg.data > 1.5:  # 如果距离大于1
            # 创建一个Twist消息，设置线速度为0.5
            twist = Twist()
            twist.linear.x = 0.3
            self._pub_cmd_vel.publish(twist)  # 发布消息
    

if __name__ == '__main__':
    rospy.init_node('tracker_status_handler')
    handler = TrackerStatusHandler()
    rospy.spin()