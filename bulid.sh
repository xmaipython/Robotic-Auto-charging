echo "开始安装相机节点和标签节点"

cd ../..
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam
sudo apt install ros-$ROS_DISTRO-apriltag-ros

echo "开始编译节点"

catkin_make -j2
