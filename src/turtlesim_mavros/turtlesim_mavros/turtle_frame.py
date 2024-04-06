from turtlesim_mavros.turtle import Turtle
import rclpy
from rclpy.node import Node

from PyQt6.QtWidgets import QFrame
from PyQt6.QtGui import QImage, QPaintEvent, QPainter, qRgb
from PyQt6.QtCore import QTimer, QPointF, QPoint

import os
from turtlesim_mavros.path_manager import IMAGES_DIR


DEFAULT_BG_R = 0x45
DEFAULT_BG_G = 0x56
DEFAULT_BG_B = 0xff

class TurtleFrame(QFrame):
    def __init__(self, nh:Node) -> None:
        super().__init__()

        window_size = (1500, 1500)
        # 类属性
        self.nh_           :Node              = nh
        self.update_timer_ :QTimer            = QTimer(self)
        self.background_   :QImage            = QImage(*window_size, QImage.Format.Format_ARGB32)
        self.frame_count_  :int               = (0)
        self.turtles_      :dict[str,Turtle]  = {}  # 乌龟表，由乌龟名索引乌龟对象实例。
        self.id_counter_   :int               = 0

        # 载入乌龟图像
        self.turtle_images_:QImage = QImage(IMAGES_DIR + os.listdir(IMAGES_DIR)[0])
        
        # 初始化UI
        self.setFixedSize(*window_size)
        self.setWindowTitle("Turtlesim")

        # 设置QT计时器, 每16ms调用一次self.__onUpdate方法，即更新频率约60Hz
        self.update_timer_.setInterval(16)
        self.update_timer_.start()
        self.update_timer_.timeout.connect(self.__onUpdate)

        # 设置meter(使用乌龟图像的高度作为比例单位)
        self.meter_ = self.turtle_images_.height()

        # 设置背景颜色
        self.background_.fill(
            qRgb(DEFAULT_BG_R, DEFAULT_BG_G, DEFAULT_BG_G)
        )
        self.update()
        
        # 以新单位计算长度
        self.width_in_meters_ = (self.width() - 1) / self.meter_
        self.height_in_meters_ = (self.height() - 1) / self.meter_

        # 生成一直乌龟
        self.spawnTurtle(self.width_in_meters_/2.0, self.height_in_meters_/2.0, 0)
        
    def spawnTurtle(self, x:float, y:float, angle:float):
        """ 以指定的姿态产生一只乌龟并返回新产生的乌龟名。若指定的乌龟名已被使用则返回空字符串，不产生新乌龟。

        Args:
            x (float): 产生的乌龟的横坐标，以self.meter_为单位
            y (float): 产生的乌龟的纵坐标，以self.meter_为单位
            angle (float): 产生的乌龟的角度
        """
        # 产生一只乌龟
        self.turtle_ = Turtle(self.nh_, self.turtle_images_, QPointF(x, self.height_in_meters_ - y), angle)
        self.update()

    def paintEvent(self, event:QPaintEvent) -> None:
        """ 重写qt paintEvent方法
            1. 绘制背景图。
            2. 绘制所有乌龟（由乌龟自己负责绘制）。
        """
        painter = QPainter(self)
        painter.drawImage(QPoint(0, 0), self.background_)
        
        self.turtle_.paint(painter)
    
    def __onUpdate(self) -> None:
        rclpy.spin_once(self.nh_, timeout_sec=0.016)
        self.__updateTurtle()
        if not rclpy.ok():
            self.close()
    
    def __updateTurtle(self):
        """更新乌龟
        """
        modified:bool = False
        modified |= self.turtle_.update(0.001 * self.update_timer_.interval(), self.width_in_meters_, self.height_in_meters_)
        if modified:
            self.update()
        self.frame_count_ += 1
    