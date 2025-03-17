#! /usr/bin/env python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool  # 导入布尔消息类型

# 定义导航函数
def start_navigation():
    # 创建 action client
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()

    # 从参数服务器获取目标点坐标
    target_x = rospy.get_param('~target_x', 6.345)  # 默认值为 6.345
    target_y = rospy.get_param('~target_y', 8.494)  # 默认值为 8.494
    target_z = rospy.get_param('~target_z', 0.0)    # 默认值为 0.0
    target_orientation_z = rospy.get_param('~target_orientation_z', -0.24)  # 默认值为 -0.24
    target_orientation_w = rospy.get_param('~target_orientation_w', 0.97)   # 默认值为 0.97

    # 设置目标点
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.position.z = target_z
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = target_orientation_z
    goal.target_pose.pose.orientation.w = target_orientation_w

    # 发送目标点
    ac.send_goal(goal)
    rospy.loginfo("开始导航...")

    # 等待导航结果
    ac.wait_for_result()

    # 判断导航是否成功
    navigation_success = ac.get_state() == actionlib.GoalStatus.SUCCEEDED
    if navigation_success:
        rospy.loginfo("导航成功！")
    else:
        rospy.loginfo("导航失败...")

    # 发布导航结果
    success_msg = Bool()
    success_msg.data = navigation_success
    navigation_success_pub.publish(success_msg)

# 定义回调函数
def start_callback(msg):
    if msg.data:  # 如果接收到的布尔值为 true
        rospy.loginfo("接收到启动信号，开始导航...")
        start_navigation()

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("nav_client")

    # 创建导航成功话题的发布者
    navigation_success_pub = rospy.Publisher('/navigation_success', Bool, queue_size=1)

    # 创建订阅者，订阅 /StartGoBaseNode 话题
    rospy.Subscriber("/StartGoBaseNode", Bool, start_callback)

    # 保持节点运行
    rospy.spin()