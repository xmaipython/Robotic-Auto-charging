#! /usr/bin/env python3
#encoding utf-8

import os
import sys

# 添加当前脚本所在目录到sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from NavWindowUI import Ui_Form
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QObject,pyqtSignal
from PyQt5.QtGui import *
import rospy
#导入项目
from call_trajectory.srv import xju_task
from call_trajectory.srv import keepOutZone

#创建自定义类，继承py中的类
class myWindow(QWidget,Ui_Form):
    def __init__(self):
        super(myWindow,self).__init__()
        #初始化UI
        self.setupUi(self)

class WindowFunction():
    def __init__(self,ui):
        self.ui=ui
        rospy.init_node('call_trajectory_ui', anonymous=True)
        rospy.wait_for_service('/xju_task')
        rospy.wait_for_service('/xju_zone')
        self.task_client = rospy.ServiceProxy('/xju_task', xju_task)
        self.zone_client = rospy.ServiceProxy('/xju_zone', keepOutZone)
        self.type_list = [0, 1]
        self.command_list = [0, 1, 2, 3]
    
    def OnBnClickDrawZone(self):
        self.ui.textEdit_Log.clear() 
        try:
            type_ = self.type_list[1] 
            command = self.command_list[0]
            dir = ""
            path_name = ""
            map = ""
            response = self.task_client(type_, command, dir, path_name, map)
            msg = f"✅[成功] 调用xju_task服务,返回消息: {response.message}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(msg)
        except Exception as e:
            err = f"[错误] xju_task服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)


    def OnBnClickCreatTeachPath(self):
        self.ui.textEdit_Log.clear() 
        try:
            type_ = self.type_list[1] 
            command = self.command_list[1]
            dir = ""
            path_name = ""
            map = ""
            response = self.task_client(type_, command, dir, path_name, map)
            msg = f"✅[成功] 调用xju_task服务,返回消息: {response.message}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")
        except Exception as e:
            err = f"[错误] xju_task服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)

    def OnBnClickCreatZPath(self):
        self.ui.textEdit_Log.clear() 
        try:
            type_ = self.type_list[1] 
            command = self.command_list[2]
            dir = ""
            path_name = ""
            map = ""
            response = self.task_client(type_, command, dir, path_name, map)
            msg = f"✅[成功] 调用xju_task服务,返回消息: {response.message}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")
        except Exception as e:
            err = f"[错误] xju_task服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)
            
    def OnBnClickCreatOPath(self):
        self.ui.textEdit_Log.clear() 
        try:
            type_ = self.type_list[1] 
            command = self.command_list[3]
            dir = ""
            path_name = ""
            map = ""
            response = self.task_client(type_, command, dir, path_name, map)
            msg = f"✅[成功] 调用xju_task服务,返回消息: {response.message}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")
        except Exception as e:
            err = f"[错误] xju_task服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)
            
    def pushBut_UsePath(self):
        self.ui.textEdit_Log.clear() 
        try:
            type_ = self.type_list[0] 
            command = self.command_list[0]
            dir = ""
            path_name = self.ui.textEdit.toPlainText()
            map = ""
            response = self.task_client(type_, command, dir, path_name, map)
            msg = f"✅[成功] 调用xju_task服务,返回消息: {response.message}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")
        except Exception as e:
            err = f"[错误] xju_task服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)
    def OnBnClickDrawWall(self):
        self.ui.textEdit_Log.clear() 
        try:
            command = 3
            cost = 0
            zone = []
            id = 0
            response = self.zone_client(command, cost, zone, id)
            msg = f"✅[成功] 开始绘制虚拟墙,返回消息: {response.id}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")

        except Exception as e:
            err = f"[错误] xju_zone服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)

    def OnBnClickuUseWall(self):
        self.ui.textEdit_Log.clear() 
        try:
            command = 4
            cost = 0
            zone = []
            id = 0
            response = self.zone_client(command, cost, zone, id)
            msg = f"✅成功生成虚拟墙,返回消息: {response.id}"
            self.ui.textEdit_Log.append(msg)
            rospy.loginfo(f"{msg}")
        except Exception as e:
            err = f"[错误] xju_zone服务调用失败: {str(e)}"
            self.ui.textEdit_Log.append(err)
            rospy.logerr(err)
  
if __name__ == '__main__':
    #1.创建主程序类
    app = QApplication(sys.argv)
    w = myWindow()
    w.show()
    myWindowFunction = WindowFunction(w)
    w.pushBut_creatTeachPath.clicked.connect(myWindowFunction.OnBnClickCreatTeachPath)
    w.pushBut_drawZone.clicked.connect(myWindowFunction.OnBnClickDrawZone)
    w.pushBut_creatZPath.clicked.connect(myWindowFunction.OnBnClickCreatZPath)
    w.pushBut_creatOPath.clicked.connect(myWindowFunction.OnBnClickCreatOPath)

    w.pushBut_UsePath.clicked.connect(myWindowFunction.pushBut_UsePath)

    w.pushBut_drawWall.clicked.connect(myWindowFunction.OnBnClickDrawWall)
    w.pushBut_useWall.clicked.connect(myWindowFunction.OnBnClickuUseWall)
 
    #2.阻塞主函数，等待APP退出
    sys.exit(app.exec_())
   

