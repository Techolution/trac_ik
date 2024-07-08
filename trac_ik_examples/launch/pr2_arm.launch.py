import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    num_samples = LaunchConfiguration('num_samples')
    chain_start = LaunchConfiguration('chain_start')
    chain_end = LaunchConfiguration('chain_end')
    timeout = LaunchConfiguration('timeout')

    # Define pose parameters
    # pos_x = LaunchConfiguration('pos_x')
    # pos_y = LaunchConfiguration('pos_y')
    # pos_z = LaunchConfiguration('pos_z')
    # orient_x = LaunchConfiguration('orient_x')
    # orient_y = LaunchConfiguration('orient_y')
    # orient_z = LaunchConfiguration('orient_z')
    # orient_w = LaunchConfiguration('orient_w')

    pkg_share = FindPackageShare('trac_ik_examples').find('trac_ik_examples')
    urdf_file = os.path.join(pkg_share, 'launch', 'pr2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument('num_samples', default_value='1'),
            DeclareLaunchArgument('chain_start', default_value='base_link_aih'),
            DeclareLaunchArgument('chain_end', default_value='link_4_left'),
            DeclareLaunchArgument('timeout', default_value='0.005'),
            # DeclareLaunchArgument('pos_x', default_value='-0.07226132922801609'),
            # DeclareLaunchArgument('pos_y', default_value='-0.03117821036125143'),
            # DeclareLaunchArgument('pos_z', default_value='0.37203747270478615'),
            # DeclareLaunchArgument('orient_x', default_value='-0.48683405994888185'),
            # DeclareLaunchArgument('orient_y', default_value='-0.49617100675010856'),
            # DeclareLaunchArgument('orient_z', default_value='-0.39504593801747606'),
            # DeclareLaunchArgument('orient_w', default_value='0.6006210427467262'),
            Node(
                package='trac_ik_examples',
                executable='ik_tests',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_desc,
                        'num_samples': num_samples,
                        'chain_start': chain_start,
                        'chain_end': chain_end,
                        'timeout': timeout,
                        # 'pos_x': pos_x,
                        # 'pos_y': pos_y,
                        # 'pos_z': pos_z,
                        # 'orient_x': orient_x,
                        # 'orient_y': orient_y,
                        # 'orient_z': orient_z,
                        # 'orient_w': orient_w,
                    }
                ],
            ),
        ]
    )
