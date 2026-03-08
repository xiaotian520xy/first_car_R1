import rclpy
from rclpy.node import Node
import qrcode
from PIL import Image
import numpy as np
import tkinter as tk
from PIL import ImageTk
from fyt_msg.msg import IntArray
from std_msgs.msg import Bool


class SendPath(Node):
    def __init__(self):
        super().__init__('send_path')
        self.path_data = None
        self.display_flag = False
        self.window_size = (500, 500)  # 固定窗口大小

        self.path_sub = self.create_subscription(
            IntArray,
            '/path_result',
            self.path_callback,
            10
        )

        self.flag_sub = self.create_subscription(
            Bool,
            '/path_flag',
            self.flag_callback,
            10
        )

        # 创建Tkinter窗口
        self.root = None
        self.label = None
        self.get_logger().info("节点已启动")

    def path_callback(self, msg):
        self.path_data = list(msg.data)
        self.display_qr_code()
        self.get_logger().info(f"收到路径数据: {self.path_data}")

    def flag_callback(self, msg):
        self.display_flag = msg.data
        self.get_logger().info(
            f"收到显示标志: {self.display_flag}, 当前数据: {self.path_data is not None}")

        # 如果标志位为True且有数据，则显示二维码
        if self.display_flag and self.path_data is not None:
            self.get_logger().info("触发显示二维码")
            self.display_qr_code()
        # 如果标志位为False，则关闭二维码界面
        elif not self.display_flag:
            self.get_logger().info("触发关闭二维码")
            self.close_qr_code()
        else:
            self.get_logger().warn(
                f"条件不满足: flag={self.display_flag}, data={self.path_data is not None}")

    def vector_to_qr(self, data_vector):
        # 将数据转换为字符串
        hex_string = ' '.join([f"{x:02X}" for x in data_vector])

        # 创建二维码
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )

        # 添加数据
        qr.add_data(hex_string)
        qr.make(fit=True)

        # 生成二维码图像
        qr_image = qr.make_image(fill_color="black", back_color="white")

        return qr_image

    def display_qr_code(self):
        if self.path_data is None:
            self.get_logger().warn("没有路径数据，无法显示二维码")
            return

        # 生成二维码
        try:
            qr_image = self.vector_to_qr(self.path_data)
            self.get_logger().info("二维码生成成功")
        except Exception as e:
            self.get_logger().error(f"二维码生成失败: {str(e)}")
            return

        # 初始化窗口
        if self.root is None:
            self.root = tk.Tk()
            self.root.title("路径二维码")
            # 设置窗口大小
            self.root.geometry(f"{self.window_size[0]}x{self.window_size[1]}")
            # 窗口居中显示
            self.center_window()
            # 禁止窗口大小调整
            self.root.resizable(False, False)
            self.get_logger().info("窗口创建成功")

        # 调整二维码大小以适应窗口
        qr_image_resized = qr_image.resize(
            (self.window_size[0] - 100, self.window_size[1] - 100),
            Image.Resampling.LANCZOS
        )

        # 转换为Tkinter图像
        photo = ImageTk.PhotoImage(qr_image_resized)

        # 创建或更新标签
        if self.label is None:
            try:
                self.label = tk.Label(self.root, image=photo)
                self.label.image = photo
                self.label.pack(expand=True)
                self.get_logger().info("标签创建成功")

                # 显示窗口
                self.root.deiconify()
                self.root.update()
                self.get_logger().info("窗口初次更新完成")

            except Exception as e:
                self.get_logger().error(f"标签创建失败: {str(e)}")
                return
        else:
            try:
                # 更新现有图像
                self.label.configure(image=photo)
                self.label.image = photo
                self.get_logger().info("图像更新成功")

                # 确保窗口显示
                self.root.deiconify()
                self.root.lift()  # 将窗口置于最前
                self.get_logger().info("窗口显示确认")

            except Exception as e:
                self.get_logger().error(f"图像更新失败: {str(e)}")

    def center_window(self):
        if self.root:
            self.root.update_idletasks()
            screen_width = self.root.winfo_screenwidth()
            screen_height = self.root.winfo_screenheight()
            x = (screen_width - self.window_size[0]) // 2
            y = (screen_height - self.window_size[1]) // 2
            self.root.geometry(
                f"{self.window_size[0]}x{self.window_size[1]}+{x}+{y}")

    def close_qr_code(self):
        if self.root is not None:
            self.root.withdraw()
            self.get_logger().info("二维码界面已关闭")


def main(args=None):
    rclpy.init(args=args)
    node = SendPath()

    try:
        # 使用非阻塞方式轮询
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # 定期更新Tkinter窗口
            if node.root:
                try:
                    node.root.update()
                except tk.TclError as e:
                    # 窗口可能已被关闭
                    node.get_logger().debug(f"窗口更新跳过: {str(e)}")
                except Exception as e:
                    node.get_logger().error(f"窗口更新失败: {str(e)}")

    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    finally:
        node.get_logger().info("正在关闭节点")

        # 关闭Tkinter窗口
        if node.root:
            try:
                node.root.quit()
                node.root.destroy()
            except Exception as e:
                node.get_logger().error(f"窗口关闭失败: {str(e)}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
