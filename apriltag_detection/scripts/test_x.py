#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def tag_callback(msg):
    """
    AprilTag 检测结果的回调函数
    :param msg: AprilTagDetectionArray 类型的消息，包含检测到的标签信息
    """
    global last_print_time

    # 获取当前时间
    current_time = rospy.Time.now()

    # 检查是否已经过了一秒
    if (current_time - last_print_time).to_sec() >= 1.0:
        # 遍历所有检测到的标签
        for detection in msg.detections:
            tag_id = detection.id[0]  # 标签 ID
            position = detection.pose.pose.pose.position  # 标签位置
            # 打印标签的 x, y, z 坐标

            rospy.loginfo(f"Tag ID: {tag_id}, Position (x, y, z): ({position.x:.4f}, {position.y:.4f}, {position.z:.4f})")

        # 更新上次打印时间
        last_print_time = current_time

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('tag_position_printer')

    # 全局变量，记录上次打印时间
    last_print_time = rospy.Time.now()

    # 订阅 AprilTag 检测结果话题
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    # 进入 ROS 主循环
    rospy.loginfo("开始打印标签位置信息...")
    rospy.spin()