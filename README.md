A ROS2 demo，在阉割版`turtlesim`中嵌入一个mavros topic `/mavros/imu/data`的subscriber，使得可以使用飞控板控制小乌龟运动。

This package is tested on Ubuntu22.04 with ROS 2 Humble.

# Usage:
1. 如果ros2和mavros环境还没搭起来可以先看看[这个文档](https://docs.qq.com/doc/DVHJaS09YeVJyZ2Fl)。

2. clone the repo
```bash
git clone 
cd turtlesim_mavros
```

3. setup ROS2 env
```bash
source /opt/ros/<ros-dist>/setup.bash
```

4. build the package
```bash
colcon build --symlink-install  # --symlink-install is necessary here, or turtle image can not be found.
```

5. setup package
```bash
source install/setup.bash
```

6. PyQt6 is needed for GUI.
```bash
pip3 install pyqt6
```

6. run the turtlesim node
```bash
ros2 run turtlesim_mavros turtlesim
```

7. run controller node  
    1. 使用键盘控制（使用[turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#use-turtlesim)包的turtle_teleop_key）
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
    2. use mavros  
    把飞控板连到电脑上，ros2运行apm.launch后就可以用飞控板控制小乌龟了。