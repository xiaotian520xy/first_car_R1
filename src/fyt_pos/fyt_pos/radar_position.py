import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros
import math
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix, quaternion_from_euler


class BasePositionPublisher(Node):
    def __init__(self):
        super().__init__('base_position_publisher')

        # 发布车体位置和 yaw
        self.position_pub = self.create_publisher(Point, '/r2_position', 10)
        self.yaw_pub = self.create_publisher(Float64, '/now_yaw', 10)

        # 订阅里程计（雷达位姿）
        self.pose_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.pose_callback,
            10
        )

        # Tf变换buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # tf变换发布器
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_tf()

        # 初始化初始位姿
        self.init_pos = None
        self.init_yaw = None

        # 卡尔慢参数
        self.x_hat = 0.0   # 初始状态估计
        self.x_P = 0.02    # 初始估计误差协方差
        self.x_Q = 0.005    # 过程噪声协方差
        self.x_R = 0.0001  # 测量噪声协方差

        self.y_hat = 0.0
        self.y_P = 0.02
        self.y_Q = 0.005
        self.y_R = 0.0001

        # 低通滤波参数
        self.alpha_x = 0.2
        self.alpha_y = 0.2
        self.last_x = 0.0
        self.last_y = 0.0

    def publish_static_tf(self):
        # 发布 base_link -> lidar_link 的静态外参
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # 父坐标系
        t.child_frame_id = 'camera_init'  # 子坐标系（雷达）

        # 雷达在车上的安装位置
        t.transform.translation.x = 0.0102  # 前+后-
        t.transform.translation.y = -0.4047  # 左+右-
        t.transform.translation.z = 0.8413  # 高度

        # 安装角度
        roll = 0.0  # 绕x轴旋转 (弧度)
        pitch = 0.0  # 绕y轴旋转 (弧度)
        yaw = math.radians(0)  # 绕z轴旋转（角度）

        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"已发布 base_link -> lidar_link 静态外参, 偏转角度: roll={roll}, pitch={pitch}, yaw={yaw:.3f} rad"
        )

    def pose_callback(self, msg: Odometry):
        try:
            # 拿静态外参：base_link -> camera_init
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_init',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # lidar在odom下的位姿
        lidar_pos = msg.pose.pose.position
        lidar_ori = msg.pose.pose.orientation

        T_odom_lidar = quaternion_matrix(
            [lidar_ori.x, lidar_ori.y, lidar_ori.z, lidar_ori.w])
        T_odom_lidar[0, 3] = lidar_pos.x
        T_odom_lidar[1, 3] = lidar_pos.y
        T_odom_lidar[2, 3] = lidar_pos.z

        R_odom_lidar = T_odom_lidar[:3, :3]
        p_odom_lidar = T_odom_lidar[:3, 3]

        org_x = lidar_pos.x
        org_y = lidar_pos.y
        org_z = lidar_pos.z
        # lidar在base下的位姿
        T_base_lidar = quaternion_matrix([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ])
        T_base_lidar[0, 3] = t.transform.translation.x
        T_base_lidar[1, 3] = t.transform.translation.y
        T_base_lidar[2, 3] = t.transform.translation.z

        R_base_lidar = T_base_lidar[:3, :3]
        p_base_lidar = T_base_lidar[:3, 3]

        # 计算base在odom下的位姿
        R_odom_base = R_odom_lidar @ R_base_lidar.T
        p_odom_base = p_odom_lidar - (R_odom_base @ p_base_lidar)
        base_pos = p_odom_base

        # 计算base的四元数与yaw（用于记录初始方向）
        T_odom_base = np.eye(4)
        T_odom_base[:3, :3] = R_odom_base
        T_odom_base[:3, 3] = p_odom_base
        q_base = quaternion_from_matrix(T_odom_base)
        yaw_base = self.quaternion_to_yaw(q_base)

        # 记录初始base_link位姿（用于转换到世界坐标系）
        if self.init_pos is None or self.init_yaw is None:
            self.init_pos = base_pos.copy()
            self.init_yaw = yaw_base
            self.get_logger().info(
                f"记录初始（上电）base_link: pos={self.init_pos}, yaw={self.init_yaw:.3f}")
            return

        # 计算相对（以上电时 base_link 为原点），并把坐标旋转到上电时的坐标系（前y，右x）
        dx = base_pos[0] - self.init_pos[0]
        dy = base_pos[1] - self.init_pos[1]
        dz = base_pos[2] - self.init_pos[2]

        # 将 dx,dy 旋转到上电时的坐标系(转换到标准右手系前x左y)
        rx = math.cos(-self.init_yaw) * dx - math.sin(-self.init_yaw) * dy
        ry = math.sin(-self.init_yaw) * dx + math.cos(-self.init_yaw) * dy
        rz = dz

        # 进行卡尔慢滤波
        rx_, self.x_hat, self.x_P = self.kalman_filter_simple(
            self.x_hat, self.x_P, self.x_Q, self.x_R, rx
        )
        ry_, self.y_hat, self.y_P = self.kalman_filter_simple(
            self.y_hat, self.y_P, self.y_Q, self.y_R, ry
        )

        # 进行低通滤波
        rx_final = self.low_pass_filter(rx, self.last_x, self.alpha_x)
        ry_final = self.low_pass_filter(ry_, self.last_y, self.alpha_y)
        self.last_x = rx_final
        self.last_y = ry_final

        # yaw归一化到（-pi，pi）
        rel_yaw = yaw_base - self.init_yaw
        rel_yaw = math.atan2(math.sin(rel_yaw), math.cos(rel_yaw))

        # 发布
        point_msg = Point(x=float(rx), y=float(ry), z=float(rz))
        self.position_pub.publish(point_msg)

        yaw_msg = Float64()
        yaw_msg.data = rel_yaw
        self.yaw_pub.publish(yaw_msg)

        # self.get_logger().info(f"base_link 相对上电位姿: x={org_x:.3f}, y={org_y:.3f}, z={org_z:.3f}, yaw={rel_yaw:.3f}")
    # 低通滤波
    def low_pass_filter(self, current, last, alpha):
        return alpha * current + (1 - alpha) * last

    def quaternion_to_yaw(self, q):
        # 四元数转 yaw (弧度)
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # 卡尔慢滤波
    def kalman_filter_simple(self, x_hat, x_P, x_Q, x_R, measurement):

        # 预测步骤
        x_hat_prior = x_hat
        P_prior = x_P + x_Q

        # 更新步骤
        K = P_prior / (P_prior + x_R)
        x_hat_new = x_hat_prior + K * (measurement - x_hat_prior)
        P_new = (1 - K) * P_prior

        return x_hat_new, x_hat_new, P_new


def main(args=None):
    rclpy.init(args=args)
    node = BasePositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
