import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. DECLARAR EL ARGUMENTO DE LA TERMINAL (IP personalizable)
    # Si no pones nada en la terminal, usará '0.0.0.0' por defecto
    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip',
        default_value='0.0.0.0',
        description='IP del servidor ROS-TCP'
    )

    # Variable que usaremos para pasarle el valor al nodo
    ros_ip = LaunchConfiguration('ros_ip')

    # 2. EL SERVIDOR TCP (ros_tcp_endpoint)
    tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='default_server_endpoint',
        parameters=[{'ROS_IP': ros_ip}], # Aquí le inyectamos la IP de la terminal
        output='screen'
    )

    # 3. EL CEREBRO DE MOVEIT (franka_fr3_moveit_config)
    # Primero buscamos dónde está instalado ese paquete en tu sistema
    moveit_launch_dir = get_package_share_directory('franka_fr3_moveit_config')
    
    # Lo lanzamos pasándole sus propios argumentos (fake hardware)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'robot_ip': 'dont-care',
            'use_fake_hardware': 'true'
        }.items()
    )

    # 4. TU NODO TRADUCTOR (unity_franka_bridge)
    planner_node = Node(
        package='unity_franka_bridge',
        executable='planner_node',
        name='planner_node',
        output='screen'
    )

    # 5. EMPAQUETAR Y LANZAR TODO
    return LaunchDescription([
        ros_ip_arg,
        tcp_endpoint_node,
        moveit_launch,
        planner_node
    ])