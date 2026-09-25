import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('delta_robot_pkg')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'delta_robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'dual_robot.rviz')

    import xacro
    robot_desc = xacro.process_file(urdf_path).toxml()

    # Ghost Robot State Publisher (/ghost namespace)
    ghost_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ghost_robot_state_publisher',
        namespace='ghost',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': 'ghost/'
        }],
        remappings=[('/joint_states', '/ghost/joint_states')]
    )

    # Real Robot State Publisher (/real namespace)
    real_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='real_robot_state_publisher',
        namespace='real',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': 'real/'
        }],
        remappings=[('/joint_states', '/real/joint_states')]
    )

    # Dual Delta Robot Simulation Node
    delta_node = Node(
        package='delta_robot_pkg',
        executable='delta_robot_node',
        name='delta_robot_solver',
        output='screen'
    )

    # Trajectory Reference Publisher Node
    traj_node = Node(
        package='delta_robot_pkg',
        executable='trajectory_publisher',
        name='delta_trajectory_publisher',
        output='screen'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        ghost_rsp,
        real_rsp,
        delta_node,
        traj_node,
        rviz_node
    ])
