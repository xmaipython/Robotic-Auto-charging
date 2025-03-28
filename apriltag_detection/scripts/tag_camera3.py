#!/usr/bin/env python3
"""
充电桩自动对接程序

功能：
1. 接收导航成功信号后开始检测AprilTag标签
2. 自动调整位置和角度对准充电桩
3. 根据距离分阶段进行精确对接
4. 包含标签丢失后的搜索功能

订阅的Topic:
1. /navigation_success (std_msgs/Bool) - 导航完成信号
2. /tag_detections (apriltag_ros/AprilTagDetectionArray) - AprilTag检测结果
3. /odom_combined (geometry_msgs/PoseWithCovarianceStamped) - 里程计信息

发布的Topic:
1. /cmd_vel (geometry_msgs/Twist) - 控制机器人运动
2. /docking_complete (std_msgs/Bool) - 对接完成信号

可调参数:
- 速度参数(LINEAR_SPEED, ANGULAR_SPEED)
- 距离阈值(DISTANCE_TO_BACK, TAG_Z_THRESHOLD)
- 对准误差阈值(ANGLE_ERROR_THRESHOLD等)
- 搜索参数(SEARCH_ROTATION_LEFT等)
"""

import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool

# ================ 可调参数 ================
LINEAR_SPEED = 0.05        # 线速度(m/s)
ANGULAR_SPEED = 0.05       # 角速度(rad/s)
DISTANCE_TO_BACK = 0.05    # 最终后退距离(5cm)
ALIGNMENT_DURATION = 1     # 对准状态维持时间(s)
ANGLE_ERROR_THRESHOLD = 0.0044      # 粗略角度误差阈值(rad)
ANGLE_ERROR_THRESHOLD_FINE = 0.0015 # 精细角度误差阈值(rad)
TARGET_DISTANCE_Y = -0.036          # 目标中心点y坐标(m)
TARGET_DISTANCE_Y_ERROR = 0.0088    # 粗略左右误差阈值(m)
TARGET_DISTANCE_Y_ERROR_FINE = 0.0035 # 精细左右误差阈值(m)
TAG_Z_THRESHOLD = 0.4      # 触发两次调整的距离阈值(m)
SEARCH_ROTATION_LEFT = math.radians(15)  # 左转角度(rad)
SEARCH_ROTATION_RIGHT = math.radians(30) # 右转角度(rad)
SEARCH_TIMEOUT = 10        # 搜索超时时间(s)
Z_P_GAIN = 1               # 距离控制的P增益
PRE_BACK_Z_THRESHOLD = 0.8       # 预后退的z值阈值(m)
PRE_BACK_SPEED = -0.2            # 预后退速度(m/s)

# ================ 状态定义 ================
STATE_WAITING_FOR_NAVIGATION = 0  # 等待导航完成
STATE_DETECTING_TAGS = 1          # 检测标签状态
STATE_ALIGNING = 2                # 角度调整状态
STATE_BACKWARD = 3                # 后退状态
STATE_FINISHED = 4                # 完成状态
STATE_SEARCHING = 5               # 寻找标签状态
STATE_FIRST_ADJUSTMENT = 6        # 第一次粗略调整
STATE_SECOND_ADJUSTMENT = 7       # 第二次精细调整
STATE_PID_BACKWARD = 8            # PID控制后退状态
STATE_PRE_BACKWARD = 9           # 预后退状态

# ================ 全局变量 ================
current_state = STATE_WAITING_FOR_NAVIGATION  # 当前状态机状态
pub = None                      # 速度指令发布对象
alignment_start_time = None     # 对准开始时间
docking_complete_pub = None     # 对接完成发布对象
search_start_time = None        # 搜索开始时间
need_fine_adjustment = False    # 是否需要精细调整标志
first_adjustment_done = False   # 第一次调整完成标志
backward_start_x = None         # 后退起始x坐标
final_backward_start_x = None   # 最终后退起始x坐标
current_z = None                # 当前检测到的z值
search_phase = 0                # 搜索阶段(0:左转,1:右转)
search_phase_start_time = None  # 当前搜索阶段开始时间
back2 = True
def get_current_z():
    """
        获取当前标签0的z值
        返回值:
        float: 标签0的z坐标,获取失败返回None
    """
    try:
        tag_detections = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=0.5)
        for detection in tag_detections.detections:
            if detection.id[0] == 0:
                return detection.pose.pose.pose.position.z
    except:
        return None
    return None

def odom_callback(msg):
    """
        里程计回调函数
        记录后退起始位置
    """
    global backward_start_x, final_backward_start_x
    if current_state == STATE_BACKWARD and backward_start_x is None:
        backward_start_x = msg.pose.pose.position.x
    elif current_state == STATE_PID_BACKWARD and final_backward_start_x is None:
        final_backward_start_x = msg.pose.pose.position.x

def navigation_success_callback(msg):
    """
        导航成功回调函数
        触发状态转换到检测标签状态
    """
    global current_state
    if current_state == STATE_WAITING_FOR_NAVIGATION and msg.data:
        current_state = STATE_DETECTING_TAGS
        rospy.loginfo("导航成功,开始检测标签")

def start_searching():
    """
        开始寻找标签
        初始化搜索状态和计时器
    """
    global current_state, search_start_time, search_phase, search_phase_start_time
    current_state = STATE_SEARCHING
    search_start_time = rospy.Time.now()
    search_phase = 0
    search_phase_start_time = rospy.Time.now()
    rospy.logwarn("未检测到标签,开始寻找...")

def perform_search():
    """
        执行搜索动作
        按阶段进行左转和右转
    """
    global current_state, search_phase, search_phase_start_time
    
    # 检查总超时
    if (rospy.Time.now() - search_start_time).to_sec() > SEARCH_TIMEOUT:
        rospy.logerr("寻找标签超时,退出程序")
        rospy.signal_shutdown("寻找标签超时")
        return
    
    cmd_vel = Twist()
    phase_elapsed = (rospy.Time.now() - search_phase_start_time).to_sec()
    
    # 左转阶段 (15度)
    if search_phase == 0:
        if phase_elapsed < 3.0:  # 3秒左转
            cmd_vel.angular.z = ANGULAR_SPEED
            rospy.loginfo("寻找阶段: 左转中...")
        else:
            search_phase = 1
            search_phase_start_time = rospy.Time.now()
            rospy.loginfo("左转完成,开始右转")
    
    # 右转阶段 (30度)
    elif search_phase == 1:
        if phase_elapsed < 6.0:  # 6秒右转
            cmd_vel.angular.z = -ANGULAR_SPEED
            rospy.loginfo("寻找阶段: 右转中...")
        else:
            # 完成一轮搜索后直接回到等待状态
            current_state = STATE_WAITING_FOR_NAVIGATION
            rospy.logwarn("完成一轮旋转仍未找到标签,返回等待状态")
            pub.publish(Twist())  # 停止运动
            return
    
    pub.publish(cmd_vel)

def tag_callback(msg):
    """
        AprilTag检测回调函数
        处理标签检测结果并控制状态转换
    """
    global current_state, alignment_start_time, need_fine_adjustment, first_adjustment_done, backward_start_x, current_z,back2
    
    # 如果在寻找过程中检测到标签
    if current_state == STATE_SEARCHING and len(msg.detections) > 0:
        current_state = STATE_DETECTING_TAGS
        rospy.loginfo("在寻找过程中检测到标签,继续对接流程")
        return
    
    if current_state not in [STATE_DETECTING_TAGS, STATE_ALIGNING, STATE_FIRST_ADJUSTMENT, STATE_SECOND_ADJUSTMENT, STATE_PRE_BACKWARD]:
        return

    cmd_vel = Twist()
    tag_positions = {}
    
    for detection in msg.detections:
        tag_id = detection.id[0]
        position = detection.pose.pose.pose.position
        tag_positions[tag_id] = position
        if tag_id == 0:
            current_z = position.z

    if 0 not in tag_positions and 1 not in tag_positions:
        rospy.logwarn("无法找到充电桩！")
        pub.publish(Twist())
        start_searching()
        return
    
    # 处理单标签情况
    elif 0 in tag_positions and 1 not in tag_positions:
        cmd_vel.linear.y = LINEAR_SPEED
    elif 1 in tag_positions and 0 not in tag_positions:
        cmd_vel.linear.y = -LINEAR_SPEED
    
    # 双标签情况
    elif 0 in tag_positions and 1 in tag_positions:
        tag0_z = tag_positions[0].z
        
        # 新增: 如果z值大于预后退阈值且未开始调整,进入预后退状态
        if tag0_z > PRE_BACK_Z_THRESHOLD and not first_adjustment_done and current_state != STATE_PRE_BACKWARD:
            current_state = STATE_PRE_BACKWARD
            rospy.loginfo(f"检测到标签0的z值({tag0_z:.3f})大于{PRE_BACK_Z_THRESHOLD},先进行预后退")
            return
        
        # 原有调整逻辑
        if tag0_z > TAG_Z_THRESHOLD and not first_adjustment_done and current_state != STATE_PRE_BACKWARD:
            if back2:
                rospy.loginfo(f"检测到标签0的z值({tag0_z:.3f})大于{TAG_Z_THRESHOLD},将进行两次调整")
                back2 = False
            need_fine_adjustment = True
            current_state = STATE_FIRST_ADJUSTMENT
        elif not need_fine_adjustment and current_state != STATE_PRE_BACKWARD:
            current_state = STATE_SECOND_ADJUSTMENT
            if back2:
                rospy.loginfo(f"检测到标签0的z值({tag0_z:.3f})小于等于{TAG_Z_THRESHOLD},直接精细调整")
                back2 = False
        
        # 计算中心点和角度差
        tag0_x = tag_positions[0].x
        tag1_x = tag_positions[1].x
        center_x = (tag0_x + tag1_x) / 2
        y_diff = tag_positions[0].z - tag_positions[1].z
        
        # 根据状态选择阈值
        angle_threshold = ANGLE_ERROR_THRESHOLD_FINE if current_state == STATE_SECOND_ADJUSTMENT else ANGLE_ERROR_THRESHOLD
        distance_error = TARGET_DISTANCE_Y_ERROR_FINE if current_state == STATE_SECOND_ADJUSTMENT else TARGET_DISTANCE_Y_ERROR
        if current_state == STATE_PRE_BACKWARD:
            return
        # 角度调整
        if abs(y_diff) > angle_threshold:
            cmd_vel.angular.z = -ANGULAR_SPEED if y_diff > 0 else ANGULAR_SPEED
            cmd_vel.linear.y = 0.0
            rospy.loginfo(f"开始角度调整,剩余：{y_diff:.3f}")
        else:
            cmd_vel.angular.z = 0.0

            # 左右调整
            if center_x < TARGET_DISTANCE_Y - distance_error:
                cmd_vel.linear.y = -(LINEAR_SPEED*0.6 if current_state == STATE_SECOND_ADJUSTMENT else LINEAR_SPEED)
                rospy.loginfo(f"开始左右调整。")
            elif center_x > TARGET_DISTANCE_Y + distance_error:
                cmd_vel.linear.y = (LINEAR_SPEED*0.6 if current_state == STATE_SECOND_ADJUSTMENT else LINEAR_SPEED)
                rospy.loginfo(f"开始左右调整。")
            else:
                cmd_vel.linear.y = 0.0

        # 检查对准状态
        if cmd_vel.angular.z == 0.0 and cmd_vel.linear.y == 0.0:
            if alignment_start_time is None:
                alignment_start_time = rospy.Time.now()
            else:
                if (rospy.Time.now() - alignment_start_time).to_sec() >= ALIGNMENT_DURATION:
                    if current_state == STATE_FIRST_ADJUSTMENT:
                        first_adjustment_done = True
                        current_state = STATE_PID_BACKWARD
                        rospy.loginfo("第一次调整完成,开始PID后退")
                    elif current_state == STATE_SECOND_ADJUSTMENT:
                        backward_start_x = None  # 重置后退起始位置
                        current_state = STATE_BACKWARD  # 进入最终后退
                        rospy.loginfo("精细调整完成,开始最终后退")
                    alignment_start_time = None
        else:
            alignment_start_time = None

    pub.publish(cmd_vel)

def pre_backward():
    """
        预后退控制
        当初始距离太远时(z>0.8),先后退到0.8以内
    """
    global current_state, current_z
    
    current_z = get_current_z()
    if current_z is None:
        rospy.logwarn("无法获取标签z值")
        return
    
    cmd_vel = Twist()
    
    if current_z > PRE_BACK_Z_THRESHOLD:
        cmd_vel.linear.x = PRE_BACK_SPEED
        rospy.loginfo(f"预后退中,z值: {current_z:.3f}, 目标: <{PRE_BACK_Z_THRESHOLD}")
    else:
        cmd_vel.linear.x = 0.0
        current_state = STATE_DETECTING_TAGS
        rospy.loginfo(f"z值已降至{current_z:.3f},开始正常调整流程")
    
    pub.publish(cmd_vel)

def pid_backward():
    """ 
        PID控制后退
        根据z值动态调整后退速度
    """
    global current_state, current_z
    
    current_z = get_current_z()
    if current_z is None:
        rospy.logwarn("无法获取标签z值")
        return
    
    cmd_vel = Twist()
    
    if current_z > TAG_Z_THRESHOLD:
        # P控制: 速度与(z - 0.4)成正比
        error = current_z - TAG_Z_THRESHOLD
        speed = -min(max(LINEAR_SPEED, Z_P_GAIN * error),0.5)
        cmd_vel.linear.x = speed
        rospy.loginfo(f"PID后退中,z值: {current_z:.3f}, 速度: {speed:.3f}")
    else:
        cmd_vel.linear.x = 0.0
        current_state = STATE_SECOND_ADJUSTMENT
        rospy.loginfo(f"z值已降至{current_z:.3f},开始精细调整")
    
    pub.publish(cmd_vel)

def precise_backward():
    """
        精确后退控制
        严格按照距离后退5cm
    """
    global current_state, backward_start_x, docking_complete_pub
    
    current_odom = rospy.wait_for_message('/odom_combined', PoseWithCovarianceStamped)
    current_x = current_odom.pose.pose.position.x
    
    if backward_start_x is None:
        backward_start_x = current_x
        rospy.loginfo(f"开始最终后退,起始位置: {backward_start_x:.3f}m,目标后退{DISTANCE_TO_BACK}m")
    
    distance_moved = abs(current_x - backward_start_x)
    remaining_distance = max(0, DISTANCE_TO_BACK - distance_moved)
    
    cmd_vel = Twist()
    
    if remaining_distance > 0:
        # 最后2cm减速
        speed = -LINEAR_SPEED * 0.5 if remaining_distance < 0.02 else -LINEAR_SPEED
        cmd_vel.linear.x = speed
        rospy.loginfo(f"最终后退中,已后退: {distance_moved:.3f}m,剩余: {remaining_distance:.3f}m")
    else:
        cmd_vel.linear.x = 0.0
        current_state = STATE_FINISHED
        rospy.loginfo("后退完成,对接结束")
        docking_complete_pub.publish(Bool(True))
        rospy.signal_shutdown("任务完成")
    
    pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('tag_follower')
    
    docking_complete_pub = rospy.Publisher('/docking_complete', Bool, queue_size=1)
    rospy.Subscriber('/navigation_success', Bool, navigation_success_callback)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
    rospy.Subscriber('/odom_combined', PoseWithCovarianceStamped, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state == STATE_PRE_BACKWARD:
            pre_backward()
        elif current_state == STATE_PID_BACKWARD:
            pid_backward()
        elif current_state == STATE_BACKWARD:
            precise_backward()
        elif current_state == STATE_SEARCHING:
            perform_search()
        rate.sleep()