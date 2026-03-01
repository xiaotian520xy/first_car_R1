import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Char


class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.connect_pub = self.create_publisher(Char, '/connect_aruco', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.connect_flag = 0

        self.camera_matrix = np.array([
            [583.82315291, 0.0, 326.40372001],
            [0.0, 583.41565744, 239.04780154],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.camera_coefficients = np.array(
            [-0.40268914, -0.00375917, -0.00145091, 0.00054633, 0.29353945],
            dtype=np.float32)

        self.marker_length = 50  

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            raise RuntimeError("无法打开摄像头")

        # 设置窗口
        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('aruco', 500, 500)

    def estimate_pose(self, corners, marker_length, camera_matrix, dist_coeffs):

        obj_points = np.zeros((4, 3), dtype=np.float32)
        half = marker_length / 2.0
        obj_points[:, :2] = np.array([
            [-half, half],
            [half, half],
            [half, -half],
            [-half, -half]
        ])

        rvecs = []
        tvecs = []

        for corner in corners:
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corner.reshape(4, 2),
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)

        return rvecs, tvecs

    def timer_callback(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("无法读取摄像头帧")
                return

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == 0:
                        # 估计位姿
                        rvecs, tvecs = self.estimate_pose(
                            [corners[i]],
                            self.marker_length,
                            self.camera_matrix,
                            self.camera_coefficients)

                        if rvecs and tvecs:
                            tvec = tvecs[0].flatten()
                            center_x = tvec[0]  # 左+右-
                            center_y = tvec[1]  # 上+下-
                            center_z = tvec[2]  # 前-后+

                            if center_x > 2.5:
                                self.connect_flag = 1
                            elif center_x < 1.5:
                                self.connect_flag = 2
                            else:
                                self.connect_flag = 3

                            # 发布消息
                            msg = Char()
                            msg.data = self.connect_flag
                            self.connect_pub.publish(msg)

                            # 绘制坐标系
                            cv2.drawFrameAxes(
                                frame,
                                self.camera_matrix,
                                self.camera_coefficients,
                                rvecs[0],
                                tvecs[0],
                                self.marker_length * 0.5)

                            # 显示位置信息
                            text = f"ID {marker_id}: ({center_x:.1f}, {center_y:.1f}, {center_z:.1f}, {self.connect_flag})mm"
                            cv2.putText(
                                frame, text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            break

            else:
                self.connect_flag = 0
                msg = Char()
                msg.data = self.connect_flag
                self.connect_pub.publish(msg)

            # 显示图像
            cv2.imshow('aruco', frame)

            # 检查按键
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("收到退出信号")
                raise KeyboardInterrupt

        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(f"错误: {str(e)}")

    def destroy(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArucoDetect()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as e:
        node.get_logger().info(f"节点终止: {e}")
    finally:
        if 'node' in locals():
            node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
