#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# --- ¡NUEVO PARA EJECUTAR! Importamos las librerías de Acción ---
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory

class UnityMoveItClient(Node):
    def __init__(self):
        super().__init__('unity_moveit_client') 
        
        self.latest_unity_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_joints_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/unity/planned_path', 
            10
        )
        
        # Cliente para PLANIFICAR (La Calculadora)
        self.planner_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # --- ¡NUEVO PARA EJECUTAR! Cliente para EJECUTAR (El Conductor) ---
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        while not self.planner_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando a que MoveIt arranque...')
            
        self.get_logger().info('¡Conectado al motor de MoveIt con éxito!')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/unity/target_pose',
            self.target_pose_callback,
            10
        )

    def robot_joints_callback(self, msg):
        # Nombres exactos de los motores del brazo que necesitamos
        arm_joint_names = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        current_positions = []
        
        # Buscamos en el mensaje de ROS solo los 7 motores que nos importan
        for name in arm_joint_names:
            if name in msg.name:
                index = msg.name.index(name)
                current_positions.append(msg.position[index])
        
        # Si hemos encontrado los 7 con éxito, actualizamos 
        if len(current_positions) == 7:
            self.latest_unity_joints = current_positions

    def target_pose_callback(self, msg):
        self.get_logger().info(f"Calculando trayectoria hacia: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")

        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'fr3_arm' 
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        
        # Velocidad reducida por seguridad (50%)
        req.motion_plan_request.max_velocity_scaling_factor = 0.5 
        req.motion_plan_request.max_acceleration_scaling_factor = 0.5

        start_state = RobotState()
        js = JointState()
        js.name = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        js.position = self.latest_unity_joints
        start_state.joint_state = js
        req.motion_plan_request.start_state = start_state

        constraint = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = msg.header.frame_id if msg.header.frame_id else "world"
        pos_constraint.link_name = "fr3_link8" 
        
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01] 
        bv.primitives.append(primitive)
        bv.primitive_poses.append(msg.pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraint.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = pos_constraint.header.frame_id
        ori_constraint.link_name = pos_constraint.link_name
        ori_constraint.orientation = msg.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1 
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        constraint.orientation_constraints.append(ori_constraint)

        req.motion_plan_request.goal_constraints.append(constraint)

        future = self.planner_client.call_async(req)
        future.add_done_callback(self.planning_response_callback)

    def planning_response_callback(self, future):
        try:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1: 
                # Extraemos la trayectoria completa para la ejecución y la conjunta para Unity
                full_trajectory = response.motion_plan_response.trajectory
                joint_trajectory = full_trajectory.joint_trajectory
                
                num_points = len(joint_trajectory.points)
                self.get_logger().info(f"✅ ¡Éxito! Trayectoria calculada de {num_points} puntos. Publicando y Ejecutando...")
                
                # 1. Se lo enviamos a Unity (Fantasma/Simulador)
                self.trajectory_pub.publish(joint_trajectory)
                
                # 2. --- ¡NUEVO PARA EJECUTAR! Mandamos la orden a los motores ---
                self.execute_plan(full_trajectory)
                
            else:
                self.get_logger().error(f"❌ MoveIt falló al calcular. Código de error: {response.motion_plan_response.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"Error al contactar con MoveIt: {e}")

    # --- ¡NUEVO PARA EJECUTAR! Función que contacta con el controlador ---
    def execute_plan(self, robot_trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ El servidor de ejecución no está disponible.')
            return

        # Construimos la meta de ejecución
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = robot_trajectory

        # Enviamos la orden de moverse
        self.execute_client.send_goal_async(goal_msg)
        self.get_logger().info("🚀 ¡Comando enviado a los motores! El robot se está moviendo.")

def main(args=None):
    rclpy.init(args=args)
    node = UnityMoveItClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()