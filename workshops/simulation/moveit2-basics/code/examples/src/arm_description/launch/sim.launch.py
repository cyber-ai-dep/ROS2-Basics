import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = FindPackageShare('arm_description').find('arm_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot_arm.urdf.xacro')

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_description = xacro.process_file(
        urdf_file,
        mappings={'controllers_yaml': controllers_yaml}
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': ['-r ', os.path.join(pkg_share, 'worlds', 'sensors_world.sdf')]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'simple_arm', '-topic', '/robot_description'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # image_bridge uses image_transport so it supports compressed streams automatically
    camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'],
        output='screen',
        parameters=[{'camera.image.compressed.jpeg_quality': 75}]
    )

    # RViz expects camera_info at /camera/image/camera_info for compressed topics
    relay_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        camera_image_bridge,
        relay_camera_info,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller]
            )
        ),
        # Delay RViz2 by 8 seconds so Gazebo can load its scene first
        # (prevents GPU memory conflict / Ogre crash)
        TimerAction(period=8.0, actions=[rviz]),
    ])
