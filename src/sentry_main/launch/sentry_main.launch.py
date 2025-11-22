import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    # launch_gazebo = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory(
    #         'sentry_description'), '/launch', '/gazebo_sim.launch.py']),
    # )

    # launch_rviz2 = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory(
    #         'sentry_navigation'), '/launch', '/navigation2.launch.py']),
    # )

    launch_gazebo = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'sentry_description','gazebo_sim.launch.py'], 
        output='screen')
    
    launch_rviz2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'sentry_navigation','navigation2.launch.py'], 
        output='screen')

    run_init_robot_pose= launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'sentry_application','init_robot_pose'], 
        output='screen')
    
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[launch_gazebo]),
        launch.actions.TimerAction(period=4.0, actions=[launch_rviz2]),
        launch.actions.TimerAction(period=6.0, actions=[run_init_robot_pose]),
    ])

    return launch.LaunchDescription([

        # launch_gazebo,
        # launch_rviz2,
        # run_init_robot_pose,

        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=launch_gazebo,
        #         on_exit=[launch_rviz2],)
        #     ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=launch_rviz2,
        #         on_exit=[run_init_robot_pose],)
        #     ),

        action_group,
    ])
