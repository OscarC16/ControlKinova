#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from kinova_apps.srv import PickAndPlace
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.group = ReentrantCallbackGroup()
        
        # Intentamos detectar el nombre correcto del gripper
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.group)
        self._gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd', callback_group=self.group)
        
        self.srv = self.create_service(PickAndPlace, 'pick_and_place', self.handle_service, callback_group=self.group)
        self.get_logger().info('Pick and Place Node: Buscando configuración de gripper...')
        self.init_timer = self.create_timer(1.0, self.initial_homing, callback_group=self.group)

    async def initial_homing(self):
        self.init_timer.cancel()
        self.get_logger().info('--- DIAGNÓSTICO DE SISTEMA ---')
        # Listamos tópicos para ayudar al usuario
        topics = self.get_topic_names_and_types()
        gripper_topics = [t[0] for t in topics if 'gripper' in t[0].lower()]
        self.get_logger().info(f'Tópicos de gripper encontrados: {gripper_topics}')
        
        self.get_logger().info('Esperando servidores...')
        self._move_group_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Homing inicial...')
        await self.send_gripper_command(0.0) # 0.0 suele ser abierto
        await self.send_joint_goal([0.0]*6)

    async def send_joint_goal(self, joint_values, speed=0.20):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = 0.05
        constraints = Constraints()
        for i, val in enumerate(joint_values):
            jc = JointConstraint(joint_name=f'joint_{i+1}', position=val, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        goal_handle = await self._move_group_client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def send_cartesian_goal(self, x, y, z, roll, pitch, yaw, speed=0.2, plan_only=False, use_path_constraints=True, tolerance=0.15):
        import math
        gripper_offset = 0.15
        
        # Compensación inteligente según orientación
        yaw_rad = math.radians(yaw)
        if abs(pitch - 90.0) < 10.0:
            # Si apunta horizontalmente (pitch ~ 90), el offset de la pinza se compensa en el plano XY (hacia atrás)
            target_x = x - gripper_offset * math.cos(yaw_rad)
            target_y = y - gripper_offset * math.sin(yaw_rad)
            target_z = z
        else:
            # Si apunta verticalmente hacia abajo, el offset se compensa en el eje Z (hacia arriba)
            target_x = x
            target_y = y
            target_z = z + gripper_offset

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 50
        goal_msg.request.allowed_planning_time = 30.0 # Más tiempo para pasos complejos
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = 0.05
        goal_msg.planning_options.plan_only = plan_only
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link'
        target_point = BoundingVolume()
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        target_point.primitives.append(primitive)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z
        target_point.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = target_point
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'end_effector_link'
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rot.as_quat()
        ori_constraint.orientation.x, ori_constraint.orientation.y, ori_constraint.orientation.z, ori_constraint.orientation.w = quat
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(name="goal", position_constraints=[pos_constraint], orientation_constraints=[ori_constraint]))

        if use_path_constraints:
            path_ori_constraint = OrientationConstraint()
            path_ori_constraint.header.frame_id = 'base_link'
            path_ori_constraint.link_name = 'end_effector_link'
            path_ori_constraint.orientation.x, path_ori_constraint.orientation.y, path_ori_constraint.orientation.z, path_ori_constraint.orientation.w = quat
            path_ori_constraint.absolute_x_axis_tolerance = tolerance
            path_ori_constraint.absolute_y_axis_tolerance = tolerance
            path_ori_constraint.absolute_z_axis_tolerance = 3.14
            path_ori_constraint.weight = 1.0
            goal_msg.request.path_constraints.orientation_constraints.append(path_ori_constraint)

        goal_handle = await self._move_group_client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def handle_service(self, request, response):
        self.get_logger().info('--- INICIANDO SECUENCIA PICK AND PLACE ---')
        # Pinza horizontal hacia afuera: Roll=0.0, Pitch=90.0, Yaw=yaw
        r_horiz, p_horiz = 0.0, 90.0
        safe_lift = 0.15 
        
        # 1. Home -> Elevado A
        self.get_logger().info('Paso 1: Aproximación a Elevado A')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a + safe_lift, r_horiz, p_horiz, request.yaw_a, use_path_constraints=False):
            return self.fail(response, "Fallo en Paso 1: Elevado A inalcanzable.")

        # 2. Bajar a A
        self.get_logger().info('Paso 2: Descenso Vertical a A')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a, r_horiz, p_horiz, request.yaw_a):
            return self.fail(response, "Fallo en Paso 2: Punto A inalcanzable.")

        # 3. Cerrar Gripper
        self.get_logger().info('Paso 3: Cerrando Gripper...')
        if not await self.send_gripper_command(0.8): # Valor más agresivo
            return self.fail(response, "Fallo en Paso 3: No se pudo cerrar el gripper.")
        time.sleep(2.0)

        # 4. Subir a Elevado A
        self.get_logger().info('Paso 4: Elevación Vertical desde A')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a + safe_lift, r_horiz, p_horiz, request.yaw_a):
            return self.fail(response, "Fallo en Paso 4: No se pudo elevar desde A.")

        # 5. Traslado a Elevado B (CON TOLERANCIA RELAJADA)
        self.get_logger().info('Paso 5: Traslado Nivelado a Elevado B')
        if not await self.send_cartesian_goal(request.x_b, request.y_b, request.z_b + safe_lift, r_horiz, p_horiz, request.yaw_b, tolerance=0.3):
            return self.fail(response, "Fallo en Paso 5: No se pudo llegar a Elevado B.")

        # 6. Bajar a B
        self.get_logger().info('Paso 6: Descenso Vertical a B')
        if not await self.send_cartesian_goal(request.x_b, request.y_b, request.z_b, r_horiz, p_horiz, request.yaw_b):
            return self.fail(response, "Fallo en Paso 6: No se pudo bajar a B.")

        # 7. Abrir Gripper
        self.get_logger().info('Paso 7: Abriendo Gripper...')
        if not await self.send_gripper_command(0.0):
            return self.fail(response, "Fallo en Paso 7: No se pudo abrir el gripper.")
        time.sleep(2.0)

        # 8. Subir a Elevado B
        self.get_logger().info('Paso 8: Retirada Vertical desde B')
        if not await self.send_cartesian_goal(request.x_b, request.y_b, request.z_b + safe_lift, r_horiz, p_horiz, request.yaw_b):
            return self.fail(response, "Fallo en Paso 8: No se pudo retirar desde B.")

        # 9. Home Final
        self.get_logger().info('Paso 9: Regreso a Home Final')
        await self.send_joint_goal([0.0]*6)

        response.success = True
        response.message = "Secuencia ejecutada paso a paso con éxito."
        return response

    async def send_gripper_command(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0
        goal_handle = await self._gripper_client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def fail(self, response, msg):
        self.get_logger().error(msg)
        response.success = False
        response.message = msg
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
