import sys
import signal
import rclpy
from rclpy.node import Node
from turtlesim_mavros import turtle_frame
from PyQt6.QtWidgets import QApplication

signal.signal(signal.SIGINT, lambda *a: print(a))
class TurtleApp(QApplication):
    def __init__(self, argv) -> None:
        super().__init__(argv)
        rclpy.init(args=argv)
        self.nh_ = Node("turtlesim")
    
    def exec(self):
        frame = turtle_frame.TurtleFrame(self.nh_)
        frame.show()
        return QApplication.exec()
    
def main():
    app = TurtleApp(sys.argv)
    signal.signal(signal.SIGINT, lambda *a: app.quit()) # 只是为了在命令行里`crtl C`能退出。
    sys.exit(app.exec())
