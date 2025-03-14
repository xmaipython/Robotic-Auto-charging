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

    '''
        x: 6.663850695173251
        y: 8.504469387587404
        z: 0.0
        orientation: 
        x: 0.0
        y: 0.0
        z: -0.24295067933526324
        w: 0.9700386422254188

    '''

    # 设置目标点
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = 6.345
    goal.target_pose.pose.position.y = 8.494
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = -0.24
    goal.target_pose.pose.orientation.w = 0.97

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