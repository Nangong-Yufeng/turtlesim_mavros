from rclpy.node import Node
from rclpy import qos
from turtlesim_interfaces.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from PyQt6.QtGui import QImage, QTransform, QPainter
from PyQt6.QtCore import QPointF

from datetime import datetime, timedelta
import math

class Turtle:
    def __init__(self, nh:Node, turtle_image:QImage, pos:QPointF, orient:float) -> None:
        self.nh_:Node = nh                          # ROS2 结点
        self.turtle_image_:QImage = turtle_image    # 乌龟图片
        self.pos_:QPointF = pos                     # 乌龟的位置
        self.orient_:float = orient                 # 乌龟的方向
        self.lin_vel_:float = (0.0)                 # 乌龟的线速度
        self.ang_vel_:float = (0.0)                 # 乌龟的角速度
        self.last_command_time_ = datetime.now()
        self.name_ = "turtle1"
        
        # 速度信息 subscriber，可用于接收ros经典示例turtlesim中turtle_teleop_key节点的控制
        self.velocity_sub_ = self.nh_.create_subscription(
            Twist,
            self.name_+"/cmd_vel",
            callback = self.__velocity_sub_callback,
            qos_profile = 0
        )

        # 速度信息 subscriber2，用于订阅mavros中/mavros/imu/data主题信息
        self.velocity_sub2_ = self.nh_.create_subscription(
            Imu,
            "/mavros/imu/data",
            callback = self.__velocity_sub2_callback,
            qos_profile = qos.qos_profile_sensor_data
        )

        # 姿态信息 publisher
        self.pose_pub_ = self.nh_.create_publisher(Pose, self.name_+"/pose", 1)

        self.meter_ = self.turtle_image_.height()
        self.__rotateImage()


    # public:
    def update(self, dt:float, canvas_width:float, canvas_height:float) -> bool:
        """ 进行乌龟的更新。

        Args:
            dt (float): δt,用来与速度相乘得到总位移的时间
            path_painter (QPainter): 用于绘图的QPainter
            path_image (QImage): 背景图像，用于提取当前乌龟位置的颜色
            canvas_width (float): 场地宽度，用于限制乌龟的运动范围
            canvas_height (float): 场地高度，用于限制乌龟的运动范围

        Returns:
            bool: _description_
        """
        
        # 如果当前的速度设置时间超过了500ms，则停止（？）
        if datetime.now() - self.last_command_time_ > timedelta(milliseconds=1000):
            self.lin_vel_ = 0.0
            self.ang_vel_ = 0.0
            return False
        
        old_orient = self.orient_
        old_pos = QPointF(self.pos_)
        # 设置移动
        self.orient_ = math.fmod(self.orient_ + self.ang_vel_ * dt, 2*math.pi)
        self.pos_.setX(self.pos_.x() + math.sin(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        self.pos_.setY(self.pos_.y() + math.cos(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        
        # print("x:",math.sin(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        # print("y:",math.cos(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)

        # 限制移动范围
        self.pos_.setX(min(max(self.pos_.x(), 0.0), canvas_width))
        self.pos_.setY(min(max(self.pos_.y(), 0.0), canvas_height))
        
        # 发布新位置信息
        p = Pose()
        p.x = self.pos_.x()
        p.y = canvas_height - self.pos_.y()
        p.theta = self.orient_
        p.linear_velocity = self.lin_vel_
        p.angular_velocity = self.ang_vel_
        self.pose_pub_.publish(p)
    
        # 如果角度变了就要重新设置一下图片旋转了。
        if self.orient_ != old_orient:
            self.__rotateImage()

        return True


    def paint(self, painter:QPainter):
        """绘制乌龟。
        """
        p:QPointF = self.pos_ * self.meter_
        p.setX(p.x() - 0.5 * self.turtle_rotated_image_.width())
        p.setY(p.y() - 0.5 * self.turtle_rotated_image_.height())
        painter.drawImage(p, self.turtle_rotated_image_)


    # private:
    def __velocity_sub_callback(self, vel: Twist):
        """`cmd_vel`话题的订阅回调
        """
        self.last_command_time_ = datetime.now()
        self.lin_vel_ = vel.linear.x
        self.ang_vel_ = vel.angular.z
    
    
    def __velocity_sub2_callback(self, vel: Imu):
        """`/mavros/imu/data`话题的订阅回调
        """
        self.last_command_time_ = datetime.now()
        self.lin_vel_ = (1 - vel.linear_acceleration.x)
        self.ang_vel_ = (-vel.linear_acceleration.y) / 2
        
    
    def __rotateImage(self):
        """用于处理乌龟图片的旋转
        """
        transform = QTransform().rotate(-self.orient_ * 180.0 / math.pi + 90.0)
        self.turtle_rotated_image_ = self.turtle_image_.transformed(transform)