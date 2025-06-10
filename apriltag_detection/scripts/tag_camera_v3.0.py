#!/usr/bin/env python3
"""
充电桩自动对接控制程序

功能概述：
1. 接收导航成功信号后开始检测AprilTag标签
2. 自动调整位置和角度对准充电桩
3. 根据距离分阶段进行精确对接
4. 包含标签丢失后的搜索功能
5. 使用行程开关精确控制最终后退距离

订阅的Topic:
1. /navigation_success (std_msgs/Bool) - 导航完成信号
2. /tag_detections (apriltag_ros/AprilTagDetectionArray) - AprilTag检测结果
3. /imu (sensor_msgs/Imu) - IMU传感器数据
4. /odom (nav_msgs_msg/Odometry) - 里程计信息
5. /BaseResponse (Float64MultiArray) - 行程开关信息

发布的Topic:
1. /cmd_vel (geometry_msgs/Twist) - 控制机器人运动
2. /BaseStationReach (std_msgs/Bool) - 对接完成信号(True表示完成)
3. /BaseCallResponseControl (std_msgs/Bool) - 继电器启动指令(True表示启动)

可调参数:
- 速度控制:
  MAX_LINEAR_SPEED: 最大线速度(m/s)
  MIN_LINEAR_SPEED: 最小线速度(m/s)
  MAX_ANGULAR_SPEED: 最大角速度(rad/s)
  MIN_ANGULAR_SPEED: 最小角速度(rad/s)
  
- 目标位置:
  TARGET_DISTANCE_Y: 目标中心点y坐标(m)
  TARGET_DISTANCE_Z: 目标停止距离(m)
  
- 控制参数:
  KP_Y: y方向比例系数
  KP_Z: z方向比例系数
  ANGLE_ERROR_THRESHOLD: 角度误差阈值(rad)
  
- 搜索参数:
  SEARCH_TIMEOUT: 搜索超时时间(s)
  FULL_ROTATION: 完整旋转角度(2pi)
"""


import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

# ================ 参数配置 ================
# 运动控制
MAX_LINEAR_SPEED = 0.18
MIN_LINEAR_SPEED = 0.016
MIN_LINEAR_SPEED_X = 0.045
MAX_ANGULAR_SPEED = 0.2
MIN_ANGULAR_SPEED = 0.016

# 目标位置
TARGET_CENTER_Y = -0.05  # 充电桩中心Y坐标
TARGET_DISTANCE_Z = 0.455  # 目标停止距离

# 控制参数
KP_X = 0.65  # X方向比例系数
KP_Y = 0.92  # Y方向比例系数
KP_Z = 3.41 # Z轴旋转比例系数
ANGLE_ERROR_THRESHOLD = 0.0011  # 角度误差阈值
POSITION_ERROR_THRESHOLD = 0.0014  # 位置误差阈值

# 超时处理
TIMEOUT_THRESHOLD = 20.0  # 行程开关超时阈值(s)

# 搜索参数
TAG_LOST_TIMEOUT = 2.0  # 标签丢失超时(s)
SEARCH_TIMEOUT = 21.0  # 搜索超时(s)
SEARCH_ROTATION_SPEED = 0.35  # 搜索旋转速度(rad/s)
FULL_ROTATION = 2*math.pi  # 完整旋转角度(2π rad)

class DockingController:
    def __init__(self):
        rospy.init_node('docking_controller')
        
        # 通信接口
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.docking_complete_pub = rospy.Publisher('/BaseStationReach', Bool, queue_size=1)
        self.jdq_pub = rospy.Publisher("/BasecallResponseControl",Bool,queue_size=1)
        # 订阅话题
        rospy.Subscriber('/navigation_success', Bool, self.nav_callback)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/BaseResponse', Float64MultiArray, self.base_response_callback)
        
        # 状态变量
        self.state = "WAITING"
        self.last_tag_time = rospy.Time.now()
        self.last_base_response_time = rospy.Time.now()
        self.start_search_time = None
        self.current_yaw = 0
        self.start_yaw = None
        self.odom_pose = None
        self.back_start_x = None
        self.switch_triggered = False
        self.tag_0 = 0  # 充电桩左侧标签ID
        self.tag_1 = 1  # 充电桩右侧标签ID
        

    # ================ 工具方法 ================
    def get_tag_by_id(self, detections, tag_id):
        """安全获取指定ID的标签"""
        for detection in detections:
            if detection.id[0] == tag_id:
                return detection.pose.pose.pose
        return None

    def stop(self):
        """停止机器人运动"""
        self.cmd_vel_pub.publish(Twist())

    def reset_to_waiting(self, reason=""):
        """重置到等待状态"""
        self.stop()
        self.state = "WAITING"
        self.start_search_time = None
        self.start_yaw = None
        self.back_start_x = None
        rospy.logwarn(f"返回等待状态. 原因: {reason}")
        if hasattr(self, 'fine_adjust_start_time'):
            del self.fine_adjust_start_time  # 清除限速模式标志
            
    def complete_docking(self):
        """完成对接流程"""
        self.stop()
        self.docking_complete_pub.publish(Bool(True))
        self.jdq_pub.publish(Bool(False))  # 发布行程开关关闭信号
        rospy.loginfo("对接完成，关闭程序")
        # rospy.signal_shutdown("对接完成")
        self.reset_to_waiting(reason="对接完成，自动重置")

    # ================ 回调函数 ================
    def base_response_callback(self, msg):
        """处理行程开关反馈"""
        self.last_base_response_time = rospy.Time.now()
        try:
            if len(msg.data) >= 2:
                self.switch_triggered = (msg.data[1] == 1.0)
                if self.switch_triggered:
                    self.stop()
                    rospy.loginfo("行程开关触发，完成对接。")
                    self.complete_docking()
            else:
                rospy.logwarn("BaseResponse 数据长度不足")
        except Exception as e:
            rospy.logerr(f"BaseResponse 处理异常: {str(e)}")

    def odom_callback(self, msg):
        """里程计数据回调"""
        try:
            self.odom_pose = msg.pose.pose.position
        except Exception as e:
            rospy.logerr(f"里程计错误: {str(e)}")
            self.reset_to_waiting("里程计异常")

    def imu_callback(self, msg):
        """IMU数据回调"""
        try:
            q = msg.orientation
            self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 
                                        1 - 2*(q.y*q.y + q.z*q.z))
        except Exception as e:
            rospy.logerr(f"IMU错误: {str(e)}")
            self.reset_to_waiting("IMU异常")

    def nav_callback(self, msg):
        """修复后的导航回调"""
        try:
            if hasattr(self, 'state') and self.state == "WAITING" and msg.data:
                self.state = "SEARCHING"
                self.start_search_time = rospy.Time.now()
                rospy.loginfo("收到导航信号，开始搜索标签")
        except Exception as e:
            rospy.logerr(f"导航回调异常: {str(e)}")
            if not hasattr(self, 'state'):
                self.state = "WAITING"

    def tag_callback(self, msg):
        """标签检测回调"""
        self.last_tag_time = rospy.Time.now()
        
        try:
            # 安全获取标签
            tag0 = self.get_tag_by_id(msg.detections, self.tag_0)
            tag1 = self.get_tag_by_id(msg.detections, self.tag_1)
            
            # 状态处理
            if self.state == "SEARCHING":
                self.handle_searching_state(msg.detections, tag0, tag1)
            elif self.state == "APPROACHING":
                self.handle_approaching_state(tag0, tag1)
            elif self.state == "FINE_ADJUSTING":
                self.handle_fine_adjusting_state(tag0, tag1)
                
        except Exception as e:
            rospy.logerr(f"标签处理异常: {str(e)}")
            self.reset_to_waiting("处理异常")

    # ================ 状态处理 ================
    def handle_searching_state(self, detections, tag0, tag1):
        """完全重构的搜索状态处理"""
        # 1. 首先检查是否超时
        if (rospy.Time.now() - self.start_search_time).to_sec() > SEARCH_TIMEOUT:
            self.reset_to_waiting("搜索超时")
            return

        # 2. 检查是否检测到双标签
        if tag0 is not None and tag1 is not None:
            self.state = "APPROACHING"
            rospy.loginfo("检测到双标签，开始接近")
            return

        # 3. 单标签处理
        if len(detections) == 1:
            tag_id = detections[0].id[0]
            cmd_vel = Twist()
            cmd_vel.angular.z = -SEARCH_ROTATION_SPEED if tag_id == self.tag_0 else SEARCH_ROTATION_SPEED
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(f"检测到单标签{tag_id}，执行定向旋转")
            return

        # 4. 无标签时的处理（关键修正部分）
        if len(detections) == 0:
            # 初始化旋转角度记录
            if self.start_yaw is None:
                self.start_yaw = self.current_yaw
                rospy.loginfo("未检测到标签，开始全向旋转搜索")

            # 计算已旋转角度（处理角度环绕）
            current_rotation = abs(self.current_yaw - self.start_yaw)
            if current_rotation > math.pi:
                current_rotation = 2*math.pi - current_rotation

            # 检查是否完成完整旋转
            if current_rotation >= FULL_ROTATION:
                rospy.logwarn("完成全周旋转未找到标签")
                self.reset_to_waiting("搜索超时")
                return

            # 持续旋转
            cmd_vel = Twist()
            cmd_vel.angular.z = SEARCH_ROTATION_SPEED
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo_throttle(1.0, f"旋转搜索中... 已旋转: {math.degrees(current_rotation):.1f}°")

    def handle_approaching_state(self, tag0, tag1):
        """处理接近状态"""
        if tag0 is None or tag1 is None:
            self.reset_to_waiting("标签丢失")
            return
        
        # 确保tag0在左，tag1在右
        if tag0.position.x > tag1.position.x:
            tag0, tag1 = tag1, tag0
        self.jdq_pub.publish(Bool(True)) 
        self.handle_base_response_timeout()
        # 计算控制参数
        center_x = (tag0.position.x + tag1.position.x)/2
        z_distance = tag0.position.z
        y_diff = tag1.position.z - tag0.position.z

        y_error = center_x - TARGET_CENTER_Y
        if abs(y_error) < POSITION_ERROR_THRESHOLD:
            y_error = 0
        z_error = z_distance - TARGET_DISTANCE_Z
        angle_error = y_diff
        angular_speed = KP_Z * angle_error
        rospy.loginfo(f"剩余距离:{z_distance}")

        cmd_vel = Twist()
        cmd_vel.linear.x = -max(min(abs(KP_X*z_error), MAX_LINEAR_SPEED), MIN_LINEAR_SPEED_X)
        cmd_vel.linear.y = max(-MAX_LINEAR_SPEED, min(KP_Y*y_error, MAX_LINEAR_SPEED))
        cmd_vel.angular.z = max(-MAX_ANGULAR_SPEED, min(angular_speed, MAX_ANGULAR_SPEED))
        
        # 状态转换检查
        if z_error < 0 : #and abs(y_error) < 0.02
            self.state = "FINE_ADJUSTING"
            rospy.loginfo("开始精细调整")
            
        self.cmd_vel_pub.publish(cmd_vel)

    def handle_fine_adjusting_state(self, tag0, tag1):
        """最终版精细调整状态处理（完全锁定限速模式）"""
        if tag0 is None or tag1 is None:
            self.reset_to_waiting("标签丢失")
            return
        
        # 确保tag0在左，tag1在右
        if tag0.position.x > tag1.position.x:
            tag0, tag1 = tag1, tag0
            
        # 计算误差
        center_x = (tag0.position.x + tag1.position.x)/2
        y_diff = tag1.position.z - tag0.position.z
        y_error = center_x - TARGET_CENTER_Y
        if abs(y_error) < POSITION_ERROR_THRESHOLD:
            y_error = 0
        angle_error = y_diff
        
        # 状态检查
        angle_ok = abs(angle_error) < ANGLE_ERROR_THRESHOLD*2.2
        pos_ok = abs(y_error) < POSITION_ERROR_THRESHOLD*2.2
        
        cmd_vel = Twist()
        LIMITED_SPEED =  0.007 # 严格限速值
        
        # 判断是否已经进入限速模式
        in_limited_mode = hasattr(self, 'fine_adjust_start_time')
        
        # 当满足条件时进入/保持限速模式
        if angle_ok and pos_ok or in_limited_mode:
            if not in_limited_mode:
                # 首次满足条件，记录时间并进入限速模式
                self.fine_adjust_start_time = rospy.Time.now()
                rospy.loginfo("进入锁定限速模式 (永久保持v_max=0.007m/s)")
            
            # 严格速度限制（完全忽略比例计算）
            if angle_error > 0:
                cmd_vel.angular.z = LIMITED_SPEED+0.004
            elif angle_error < 0:
                cmd_vel.angular.z = -LIMITED_SPEED-0.004
            else:
                cmd_vel.angular.z = 0
            
            if y_error > 0:
                cmd_vel.linear.y = LIMITED_SPEED
            elif y_error < 0:
                cmd_vel.linear.y = -LIMITED_SPEED
            else:
                cmd_vel.linear.y = 0
            
            # 持续时间检查（仅当持续满足条件时才转换状态）
            if angle_ok and pos_ok:
                duration = (rospy.Time.now() - self.fine_adjust_start_time).to_sec()
                if duration >= 0.6:
                    self.state = "FINAL_BACK"
                    if self.odom_pose:
                        self.back_start_x = self.odom_pose.x
                    rospy.loginfo(f"精确定位完成，开始最终后退 (持续{duration:.2f}秒)")
                    del self.fine_adjust_start_time
        else:
            # 正常比例控制模式（仅在从未进入过限速模式时执行）
            if not in_limited_mode:
                if abs(angle_error) > ANGLE_ERROR_THRESHOLD:
                    angular_speed = KP_Z * angle_error
                    cmd_vel.angular.z = max(-MAX_ANGULAR_SPEED, 
                                        min(angular_speed, MAX_ANGULAR_SPEED))
                    if abs(cmd_vel.angular.z) < MIN_ANGULAR_SPEED:
                        cmd_vel.angular.z = math.copysign(MIN_ANGULAR_SPEED, angle_error)
                
                if abs(y_error) > POSITION_ERROR_THRESHOLD:
                    linear_y = KP_Y * y_error
                    cmd_vel.linear.y = max(-MAX_LINEAR_SPEED,
                                        min(linear_y, MAX_LINEAR_SPEED))
                    if abs(cmd_vel.linear.y) < MIN_LINEAR_SPEED:
                        cmd_vel.linear.y = math.copysign(MIN_LINEAR_SPEED, y_error)
        
        # 强制确保不超过限速（最终保护）
        if in_limited_mode:
            cmd_vel.angular.z = max(-LIMITED_SPEED, min(cmd_vel.angular.z, LIMITED_SPEED))
            cmd_vel.linear.y = max(-LIMITED_SPEED, min(cmd_vel.linear.y, LIMITED_SPEED))
        
        rospy.loginfo(f"[模式:{'锁定限速' if in_limited_mode else '正常'}] "
                    f"Y误差:{y_error:.4f} 角度误差:{angle_error:.4f} | "
                    f"Y速度:{cmd_vel.linear.y:.3f} Z旋转:{cmd_vel.angular.z:.3f}")
        
        self.cmd_vel_pub.publish(cmd_vel)

    # ================ 主循环处理 ================
    def run(self):
        """主控制循环"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.handle_timeouts()
                self.handle_final_back()
            except Exception as e:
                rospy.logerr(f"主循环异常: {str(e)}")
                self.reset_to_waiting("主循环异常")
            rate.sleep()

    def handle_timeouts(self):
        """处理各类超时"""
        if self.state == "SEARCHING":
            self.handle_search_timeout()
        elif self.state in ["APPROACHING", "FINE_ADJUSTING"]:
            self.handle_tag_lost_timeout()

    def handle_search_timeout(self):
        """处理搜索超时"""
        if (rospy.Time.now() - self.start_search_time).to_sec() > SEARCH_TIMEOUT:
            self.reset_to_waiting("搜索超时")
        elif not self.start_yaw:
            self.start_rotation_search()

    def handle_base_response_timeout(self):
        """处理BaseResponse继电器消息丢失超时"""
        
        time_since_last = (rospy.Time.now() - self.last_base_response_time).to_sec()
        if time_since_last > TIMEOUT_THRESHOLD:
            self.reset_to_waiting("继电器状态异常")

    def start_rotation_search(self):
        """开始旋转搜索"""
        self.start_yaw = self.current_yaw
        cmd_vel = Twist()
        cmd_vel.angular.z = SEARCH_ROTATION_SPEED
        self.cmd_vel_pub.publish(cmd_vel)

    def handle_tag_lost_timeout(self):
        """处理标签丢失超时"""
        if (rospy.Time.now() - self.last_tag_time).to_sec() > TAG_LOST_TIMEOUT:
            self.reset_to_waiting("标签丢失超时")

    def handle_final_back(self):
        """处理最终后退阶段"""
        if self.state != "FINAL_BACK":
            return
            
        if not self.odom_pose or self.back_start_x is None:
            self.reset_to_waiting("里程计数据无效")
            return
            
        distance_moved = abs(self.odom_pose.x - self.back_start_x)
        if self.switch_triggered:
            self.complete_docking()
        else:
            speed = -0.04 if (distance_moved> 0.05) else -0.06
            rospy.loginfo(f"后退距离:{distance_moved:.3f}m")
            cmd_vel = Twist()
            cmd_vel.linear.x = speed
            self.cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        controller = DockingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序启动异常: {str(e)}")