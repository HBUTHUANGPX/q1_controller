from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # 获取启动参数
    model_name = LaunchConfiguration('model_name')
    reference_frame = LaunchConfiguration('reference_frame')
    udp_port = LaunchConfiguration('udp_port')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    launch_zmq_bridge = LaunchConfiguration('launch_zmq_bridge')
    zmq_bind_address = LaunchConfiguration('zmq_bind_address')
    zmq_topic = LaunchConfiguration('zmq_topic')
    zmq_sndhwm = LaunchConfiguration('zmq_sndhwm')
    zmq_conflate = LaunchConfiguration('zmq_conflate')

    # 解析rviz_config_file路径
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('xsens_mvn_ros2'),
        'rviz',
        'xsens_visualization.rviz'
    ])

    # 定义XSens客户端节点
    xsens_client_node = Node(
        package='xsens_mvn_ros2',
        executable='xsens_client',
        name='xsens',
        output='screen',
        parameters=[
            {'model_name': model_name},
            {'reference_frame': reference_frame},
            {'udp_port': udp_port}
        ]
    )

    # 定义RViz2节点（条件启动）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='xsens_rviz',
        output='screen',
        parameters=[{'display_config': rviz_config_file}],
        condition=IfCondition(launch_rviz)
    )

    zmq_bridge_node = Node(
        package='xsens_mvn_ros2',
        executable='link_states_zmq_bridge',
        name='link_states_zmq_bridge',
        output='screen',
        parameters=[
            {'ros_topic': '/link_states'},
            {'zmq_bind_address': zmq_bind_address},
            {'zmq_topic': zmq_topic},
            {'sndhwm': zmq_sndhwm},
            {'conflate': zmq_conflate},
        ],
        condition=IfCondition(launch_zmq_bridge)
    )

    return [xsens_client_node, rviz_node, zmq_bridge_node]

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'model_name',
            default_value='skeleton',
            description='Model name for XSens client'
        ),
        DeclareLaunchArgument(
            'reference_frame',
            default_value='world',
            description='Reference frame for XSens client'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9763',
            description='UDP port for XSens communication'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('xsens_mvn_ros2'),
                'rviz',
                'xsens_visualization.rviz'
            ]),
            description='Path to RViz2 configuration file'
        ),
        DeclareLaunchArgument(
            'launch_zmq_bridge',
            default_value='false',
            description='Whether to launch the /link_states ZMQ bridge'
        ),
        DeclareLaunchArgument(
            'zmq_bind_address',
            default_value='tcp://*:5555',
            description='ZMQ bind address for the link states bridge'
        ),
        DeclareLaunchArgument(
            'zmq_topic',
            default_value='xsens.link_states.v1',
            description='ZMQ PUB topic prefix for link state payloads'
        ),
        DeclareLaunchArgument(
            'zmq_sndhwm',
            default_value='5',
            description='ZMQ send high water mark'
        ),
        DeclareLaunchArgument(
            'zmq_conflate',
            default_value='false',
            description='Whether to keep only the latest pending frame in the PUB socket'
        ),
        # 使用OpaqueFunction调用launch_setup
        OpaqueFunction(function=launch_setup)
    ])
