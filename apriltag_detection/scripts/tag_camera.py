#!/usr/bin/env python3
import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

# 定义参数
LINEAR_SPEED = 0.05  # 左右移动速度
ANGULAR_SPEED = 0.05  # 角度调整速度
DISTANCE_TO_BACK = 0.05  # 需要后退的距离
ALIGNMENT_DURATION = 1  # 角度和左右对准状态需持续的时间（秒）
ANGLE_ERROR_THRESHOLD = 0.0020
TARGET_DISTANCE_Y = -0.0580  # 让标签0处于这个位置就是标签与相机对准
TARGET_DISTANCE_Y_ERROR = 0.005
# 状态定义
STATE_WAITING_FOR_NAVIGATION = 0  # 等待导航完成状态
STATE_DETECTING_TAGS = 1  # 检测标签状态
STATE_ALIGNING = 2  # 角度和左右调整状态
STATE_BACKWARD = 3  # 后退状态
STATE_FINISHED = 4  # 完成状态

# 全局变量
current_state = STATE_WAITING_FOR_NAVIGATION  # 当前状态
pub = None  # 速度指令发布者
alignment_start_time = None  # 角度和左右对准状态开始时间
initial_odom_x = None  # 记录初始的 odom 的 x 距离
docking_complete_pub = None  # 对接完成话题发布者

def odom_callback(msg):
    """
    odom 话题的回调函数，用于记录初始的 odom 的 x 距离
    :param msg: PoseWithCovarianceStamped 类型的消息，包含机器人的里程计信息
    """
    global initial_odom_x, current_state
    if current_state == STATE_BACKWARD and initial_odom_x is None:
        initial_odom_x = msg.pose.pose.position.x

def navigation_success_callback(msg):
    """
    导航成功话题的回调函数，用于判断是否导航完成
    :param msg: Bool 类型的消息，包含导航成功的信息
    """
    global current_state
    if current_state == STATE_WAITING_FOR_NAVIGATION and msg.data:
        current_state = STATE_DETECTING_TAGS
        rospy.loginfo("导航成功，开始检测标签")

def tag_callback(msg):
    """
    AprilTag 检测结果的回调函数
    :param msg: AprilTagDetectionArray 类型的消息，包含检测到的标签信息
    """
    global current_state, alignment_start_time 
    #rospy.logwarn(f"current_state: {current_state}")
    if current_state != STATE_DETECTING_TAGS and current_state != STATE_ALIGNING:
        return  # 非检测和调整状态，不处理标签信息

    cmd_vel = Twist()  # 初始化速度指令

    # 提取检测到的标签位置
    tag_positions = {}
    for detection in msg.detections:
        tag_id = detection.id[0]  # 标签 ID
        position = detection.pose.pose.pose.position  # 标签位置
        tag_positions[tag_id] = position  # 存储整个位置对象

    # 检查是否检测到标签 0 和 1
    if 0 not in tag_positions and 1 not in tag_positions:
        rospy.logwarn("无法找到充电桩！")
        pub.publish(Twist())  # 发布速度为 0 的指令，停止机器人
        return
    elif 0 in tag_positions and 1 not in tag_positions:
        rospy.loginfo("只检测到标签 0，向左移动")
        cmd_vel.linear.y = LINEAR_SPEED  # 向左移动
    elif 1 in tag_positions and 0 not in tag_positions:
        rospy.loginfo("只检测到标签 1，向右移动")
        cmd_vel.linear.y = -LINEAR_SPEED  # 向右移动
    elif 0 in tag_positions and 1 in tag_positions:
        rospy.loginfo("检测到标签 0 和 1，开始角度调整")
        current_state = STATE_ALIGNING

        # 获取标签 0 和 1 的 x 坐标
        tag0_x = tag_positions[0].x  # 标签 0 的 x 坐标
        # tag1_x = tag_positions[1].x  # 标签 1 的 x 坐标
        # rospy.loginfo(f"tag0_x{tag0_x}")
        # rospy.loginfo(f"tag1_x{tag1_x}")
        # 计算标签 0 和 1 的中心点
        # center_x = (tag0_x + tag1_x) / 2

        # 调整角度
        tag0_z = tag_positions[0].z
        tag1_z = tag_positions[1].z
        height_diff = tag0_z - tag1_z
        rospy.loginfo(f"height_diff{height_diff:.3f}")
        
        if abs(height_diff) > ANGLE_ERROR_THRESHOLD:
            # 根据高度差调整角速度
            cmd_vel.angular.z = -ANGULAR_SPEED if height_diff > 0 else ANGULAR_SPEED
            cmd_vel.linear.y = 0.0  # 停止左右调整
            rospy.loginfo("开始调整角度...")
        else:
            cmd_vel.angular.z = 0.0  # 角度对准完成
            rospy.loginfo("角度调整完成，开始左右调整...")
            # 左右调整
            if tag0_x < TARGET_DISTANCE_Y - TARGET_DISTANCE_Y_ERROR:
                cmd_vel.linear.y = -LINEAR_SPEED
            elif tag0_x > TARGET_DISTANCE_Y + TARGET_DISTANCE_Y_ERROR:
                cmd_vel.linear.y = LINEAR_SPEED
            else:
                cmd_vel.linear.y = 0.0  # 左右对准完成

        # 判断是否处于角度和左右对准状态
        if cmd_vel.angular.z == 0.0 and cmd_vel.linear.y == 0.0:
            if alignment_start_time is None:
                alignment_start_time = rospy.Time.now()
            else:
                elapsed_time = (rospy.Time.now() - alignment_start_time).to_sec()
                if elapsed_time >= ALIGNMENT_DURATION:
                    current_state = STATE_BACKWARD
                    rospy.loginfo("角度调整完成，开始后退")
                    alignment_start_time = None
        else:
            alignment_start_time = None

    # 发布速度指令
    pub.publish(cmd_vel)

def check_backward_distance():
    """
    检查后退距离是否达到目标
    """
    global current_state, initial_odom_x

    if current_state == STATE_BACKWARD and initial_odom_x is not None:
        current_odom_x = rospy.wait_for_message('/odom_combined', PoseWithCovarianceStamped).pose.pose.position.x
        distance_traveled = abs(initial_odom_x - current_odom_x)
        if distance_traveled >= DISTANCE_TO_BACK:
            current_state = STATE_FINISHED
            rospy.loginfo("对接完成，任务结束")
            # 发布对接完成消息
            docking_complete_msg = Bool()
            docking_complete_msg.data = True
            docking_complete_pub.publish(docking_complete_msg)
            # 关闭程序
            rospy.signal_shutdown("任务完成，关闭程序")

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('tag_follower')

    # 创建对接完成话题发布者
    docking_complete_pub = rospy.Publisher('/docking_complete', Bool, queue_size=1)

    # 订阅导航成功话题
    rospy.Subscriber('/navigation_success', Bool, navigation_success_callback)

    # 订阅 AprilTag 检测结果话题
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    # 订阅 odom 话题
    rospy.Subscriber('/odom_combined', PoseWithCovarianceStamped, odom_callback)

    # 发布速度指令话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # 进入 ROS 主循环
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if current_state == STATE_BACKWARD:
            # 发布后退的速度指令
            cmd_vel = Twist()
            cmd_vel.linear.x = -LINEAR_SPEED  # 后退
            pub.publish(cmd_vel)
            check_backward_distance()
        rate.sleep()