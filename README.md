功能大部分来源于@冰达机器人，仅供免费使用，如有侵权请联系

使用前准备-功能包
```sh
$ sudo apt install ros-$ROS_DISTRO-apriltag-ros
$ 
$ cd ~/catkin_ws/src
$ git clone https://github.com/xmaipython/Robotic-Auto-charging.git
```
将文件夹apriltag_detection移动到上一个目录，然后编译工作空间
```sh
$ cd ~/catkin_ws```
$ catkin_make```
$ source ./devel/setup.bash
```

doc目录
存放了tag36h11标签族的标签图像，方便直接取用
config目录
存放相机标定文件ost.yaml和apriltag_ros相关的配置文件
其中settings.yaml文件配置了apriltag检测的标签类型、使用计算机资源等，完整的参数参考wiki链接
tags.yaml文件存放所检测的标签序号、尺寸信息，这里我们设置0、1三个标签的信息
standalone_tags:
  [
    {id: 0, size: 0.05},
    {id: 1, size: 0.05},
  ]

使用前准备-标签
将doc目录中的tag.docx文件按照一比一打印在A4纸上，如有条件，可以将打印后的A4纸贴在亚克力板或者硬纸板上，避免二维码因为纸张弯曲大幅度变形

运行自动充电程序

尝试启动相机和apriltag检测程序

```sh
$ roslaunch apriltag_detection change_assemble.launch
```

/***************change_assemble.launch****************/
nav_pose.py      # 导航节点
tag.launch        # 包含相机节点camera.launch、标签检测节点continuous_detection.launch
tag_camera.py  # 对接充电桩节点

/**********************nav_pose.py**********************/
主要功能：
	订阅 /StartGoBaseNode 话题，等待接收布尔类型的启动信号。
	当接收到 True 信号时，启动机器人导航至预设目标点。
	导航结束后，判断导航是否成功，并将结果通过 /navigation_success 话题发布出去。
设置参数：
	需要根据基站位置，从/amcl_pose获取坐标信息，来设置goal导航目标点。

/********************tag_camera.py********************/
主要功能：
	用于控制机器人在导航到指定位置后，通过检测 AprilTag 标签实现与充电桩的对接。具体流程如下：
	等待导航完成：程序启动后，处于等待状态，监听/navigation_success话题，当接收到导航成功的信号后，开始检测 AprilTag 标签。
	检测 AprilTag 标签：订阅/tag_detections话题，根据检测到的标签（标签 0 和标签 1）情况，控制机器人左右移动，使机器人大致对准充电桩。
	角度和左右调整：若同时检测到标签 0 和标签 1，开始进行角度和左右位置的精确调整，确保机器人与充电桩完全对准。
	后退对接：当角度和左右位置都对准并持续一定时间后，机器人开始后退，直到达到预设的后退距离。
	任务完成：后退完成后，发布对接完成的消息/docking_complete，并关闭程序。
设置参数：
	DISTANCE_TO_BACK：机器人后退对接的距离，默认值为0.05(m)。
	ALIGNMENT_DURATION：角度和左右对准状态需要持续的时间，单位为秒，默认值为1（s）。
	ANGLE_ERROR_THRESHOLD：角度调整的误差阈值，当标签高度差小于此值时，认为角度已对准，默认值为0.002（单位坐标）。（注：检测的tf空间与实际空间的单位并不符合，且关系非线性，故无法确定准确单位）
	TARGET_DISTANCE_Y：标签 0 的目标y坐标，当标签 0 的y坐标在此范围内时，认为左右已对准，默认值为-0.058（单位坐标）。
	TARGET_DISTANCE_Y_ERROR：左右对准的误差范围，默认值为0.005（单位坐标）。

/********************camera.launch********************/
主要功能：
	启动相机，发布/usb_cam 系列图像话题
设置参数：
	相机设备号：默认值为/dev/video0

/*************continuous_detection.launch*************/
主要功能：
	启动标签检测，并发布/tag_detections标签信息话题、/tag_detections 系列图像话题
设置参数：
	标签参数设置：路径/apriltag_detection/tags.yaml中的standalone_tags设置所检测标签序号以及标签大小，用于标定，现使用5cm边长的标签。

通过订阅tag_detections话题或者监听相机坐标相对于标签坐标之间的位置关系，就可以获得标签和相机之间的位置关系，有了这个比较准确的位置关系，可以完成目标跟踪、视觉抓取等应用。