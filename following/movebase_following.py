import rospy
import tf2_ros
import numpy as np
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Float32

# 初始化x和y的值
x = 0.0
y = 0.0

# 创建订阅者的回调函数
def distance_callback(msg):
    global x
    x = msg.data

def offset_callback(msg):
    global y
    y = msg.data

def transform_pose_to_map():
    # 创建tf Buffer
    tf_buffer = tf2_ros.Buffer()
    # 创建tf监听器
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建订阅者
    rospy.Subscriber("/tracker/distance", Float32, distance_callback)
    rospy.Subscriber("/tracker/offset", Float32, offset_callback)

    # 等待tf树中的转换关系建立好
    rospy.sleep(0.5)
    # 计算原点到点 (x, -y, 0) 的单位向量
    norm = np.sqrt(x**2 + y**2)
    unit_vector = np.array([x, -y]) / norm
    if norm <= 1.2:
        print("already close to the target")
        return

    # 计算距离点 (x, -y, 0) 为1的点的坐标
    new_point = unit_vector * (norm - 1)
    # 计算从(0,0)到(x,-y)的角度
    angle = np.arctan2(-y, x)

    # 使用tf库的Quaternion函数将角度转换为四元数
    quaternion = tf_trans.quaternion_from_euler(0, 0, angle)
    # 创建一个PoseWithCovarianceStamped消息，该消息包含物体在camera_link坐标系中的位置和方向
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "tb3_0/camera_depth_frame"
    pose_msg.pose.pose = Pose(Point(new_point[0], new_point[1], 0), Quaternion(*quaternion))

    try:
        # 获取camera_link到map的变换关系
        transform = tf_buffer.lookup_transform("map", pose_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # 使用tf2库将位姿信息从camera_link坐标系转换到map坐标系
        transformed_pose = do_transform_pose(pose_msg.pose, transform)

        # 创建一个PoseStamped消息，用于发布到/tb3_0/move_base_simple/goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = transformed_pose.pose

        # 创建一个Publisher，发布到/tb3_0/move_base_simple/goal
        pub = rospy.Publisher('/tb3_0/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(0.5)  # 等待Publisher建立连接
        pub.publish(goal_msg)

        # 处理转换后的位姿信息，可以根据需要进行后续操作
        print("Transformed Pose in map frame:")
        print(transformed_pose)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform pose: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    while not rospy.is_shutdown():
        transform_pose_to_map()
        rospy.sleep(0.7)  # 每秒更新一次
    rospy.spin()