import rclpy
from rclpy.node import Node
from std_msgs.msg import Char
import cv2
import numpy as np


class PictureShow(Node):
    def __init__(self):
        super().__init__('show_aruco')
        self.pic0 = "/home/upre/r1_2025/src/fyt_pos/picture/pic0.png"
        self.pic1 = "/home/upre/r1_2025/src/fyt_pos/picture/pic1.png"
        self.pic2 = "/home/upre/r1_2025/src/fyt_pos/picture/pic2.png"

        self.pic0_img = cv2.imread(self.pic0)
        self.pic1_img = cv2.imread(self.pic1)
        self.pic2_img = cv2.imread(self.pic2)

        # 当前显示的图片
        self.current_img = self.pic1_img if self.pic0_img is not None else np.zeros(
            (720, 1280, 3), dtype=np.uint8)

        # 创建窗口
        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('aruco', 1280, 720)

        # 初始显示
        cv2.imshow('aruco', self.current_img)
        cv2.waitKey(1)  # 初始刷新

        # 订阅
        self.pose_sub = self.create_subscription(
            Char,
            '/aruco_flag',
            self.aruco_callback,
            10
        )

        # 定时器用于更新窗口
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.need_update = False

        self.get_logger().info("图片显示节点已启动")

    def aruco_callback(self, msg):
        flag_ = msg.data
        self.get_logger().info(f"收到标志: {flag_}")

        # 根据标志选择图片
        if flag_ == 0:
            new_img = self.pic0_img
        elif flag_ == 1:
            new_img = self.pic1_img
        elif flag_ == 2:
            new_img = self.pic2_img
        else:
            new_img = self.pic1_img

        # 更新当前图片
        if new_img is not None:
            self.current_img = new_img
            self.need_update = True
        else:
            self.get_logger().warn(f"图片未加载，flag={flag_}")
            self.current_img = np.zeros((720, 1280, 3), dtype=np.uint8)
            self.need_update = True

    def timer_callback(self):
        """定时器回调，更新窗口"""
        if self.need_update:
            cv2.imshow('aruco', self.current_img)
            self.need_update = False

        # 非阻塞的按键检查
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("收到退出信号")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PictureShow()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    finally:
        node.get_logger().info("正在关闭节点")
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
