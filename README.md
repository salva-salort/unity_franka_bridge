# unity_franka_bridge

Compilar paquete:
colcon build --packages-select unity_franka_bridge

Ejecutar por separado:
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
ros2 run unity_franka_bridge planner_node

Comando launch (lanza todo):

ros2 launch unity_franka_bridge unity_system.launch.py

ros2 launch unity_franka_bridge unity_system.launch.py ros_ip:=IP
