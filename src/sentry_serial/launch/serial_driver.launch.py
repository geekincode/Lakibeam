from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类- ----- ---
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取--------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关----------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关-----------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关-----------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

# 加载显示机器人模型
def generate_launch_description():

    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='sentry_serial').find('sentry_serial')

    # 启动串口
    serial_driver = Node(
        package='sentry_serial',
        executable='sentry_serial_driver'
    )

    # base_link to laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0','0','0.18','0','0','0','base_link','laser']
    )

    # base_link to imu tf node
    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0','0','0.18','0','0','0','base_link','imu']
    )

    # 机器人定位节点
    # robot_localization_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    # )


    return LaunchDescription([serial_driver,base_link_to_laser_tf_node,base_link_to_imu_tf_node])