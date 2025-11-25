from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('q1_controller')  # 替换为您的包名
    urdf_path = os.path.join(package_dir, 'urdf', 'Q1','urdf','Q1_wo_hand.urdf')  # URDF 文件路径
    rviz_config_path = os.path.join(package_dir, 'rviz', 'rviz.rviz')  # RViz 配置文件路径（需手动创建）
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path, 'r').read()}]  # 加载 URDF
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # 加载 RViz 配置
        ),
        # Node(
        #     package='q1_controller',
        #     executable='bringup_mujoco.py',
        #     name='bringup_mujoco',
        #     output='screen'
        # ),
    ])
