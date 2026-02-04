from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 获取 Livox 驱动包的路径
    livox_package_name = "livox_ros_driver2"
    livox_package_path = get_package_share_directory(livox_package_name)
    livox_launch_path = os.path.join(
        livox_package_path, "launch_ROS2", "msg_MID360_launch.py")

    # 获取 Fast-LIO 包的路径
    fastlio_package_name = "fast_lio"
    fastlio_package_path = get_package_share_directory(fastlio_package_name)
    fastlio_launch_path = os.path.join(
        fastlio_package_path, "launch", "mapping.launch.py")

    # 定义要启动的节点

    send_node = ExecuteProcess(
        cmd=['ros2', 'run', 'chuankou', 'send'],
        output='screen'
    )

    recieve_node = ExecuteProcess(
        cmd=['ros2', 'run', 'chuankou', 'receive'],
        output='screen'
    )
    
    position_publisher_node = ExecuteProcess(
        cmd=['ros2', 'run', 'fyt_pos', 'radar_position'],
        output='screen'
    )
    
    aruco_node = ExecuteProcess(
        cmd=['ros2', 'run', 'fyt_pos', 'picture'],
        output='screen'
    )
    
    connect_node = ExecuteProcess(
        cmd=['ros2', 'run', 'fyt_pos', 'aruco'],
        output='screen'
    )

    # 定义要包含的 launch 文件

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path)
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch_path)
    )

    # 创建 LaunchDescription 对象，并添加要启动的节点和 launch 文件
    ld = LaunchDescription()
    # 1. 首先启动传感器驱动
    ld.add_action(livox_launch)
    # 2. 启动感知算法节点
    ld.add_action(fastlio_launch)
    # 3. 启动定位和决策节点
    ld.add_action(position_publisher_node)
    ld.add_action(aruco_node)
    ld.add_action(connect_node)
    # 4. 最后启动执行节点
    ld.add_action(send_node)
    ld.add_action(recieve_node)

    return ld
