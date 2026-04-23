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
        
        # Callback group for parallel execution
        self.group = ReentrantCallbackGroup()
        
        # Action Client for MoveIt 2 (Arm)
        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.group)
            
        # Action Client for Gripper
        self._gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd', callback_group=self.group)
        
        # Service
        self.srv = self.create_service(
            PickAndPlace, 'pick_and_place', self.handle_service, callback_group=self.group)
        
        self.get_logger().info('Pick and Place Node started.')
        self.init_timer = self.create_timer(1.0, self.initial_homing, callback_group=self.group)

    async def initial_homing(self):
        self.init_timer.cancel()
        self.get_logger().info('Esperando a que los servidores de accion esten listos...')
        self._move_group_client.wait_for_server()
        self._gripper_client.wait_for_server()
        
        self.get_logger().info('Homing inicial: Moviendo a Home y abriendo pinza...')
        await self.send_gripper_command(0.0) # 0.0 suele ser abierto
        await self.send_joint_goal([0.0]*6)

    async def send_joint_goal(self, joint_values, speed=0.15):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        
        constraints = Constraints()
        for i, val in enumerate(joint_values):
            jc = JointConstraint(joint_name=f'joint_{i+1}', position=val, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_handle = await self._move_group_client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def send_cartesian_goal(self, x, y, z, roll, pitch, yaw, speed=0.15):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = speed
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link'
        target_point = BoundingVolume()
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]) # Precision de 5mm
        target_point.primitives.append(primitive)
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_point.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = target_point
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'end_effector_link'
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rot.as_quat()
        ori_constraint.orientation.x, ori_constraint.orientation.y, ori_constraint.orientation.z, ori_constraint.orientation.w = quat
        # Tolerancias mas estrictas (aprox 11 grados)
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(name="goal", position_constraints=[pos_constraint], orientation_constraints=[ori_constraint]))
        goal_handle = await self._move_group_client.send_goal_async(goal_msg)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def send_gripper_command(self, position):
        """0.0 es abierto, 0.8 es cerrado para Robotiq 2F-85"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0
        
        self.get_logger().info(f'Enviando comando a la pinza: {position}')
        goal_handle = await self._gripper_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Comando de pinza rechazado')
            return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def handle_service(self, request, response):
        z_lift_offset = 0.15 # Elevacin de 15cm
        
        self.get_logger().info('Iniciando secuencia Pick and Place...')

        # 1. Aproximacin a A (Elevado)
        self.get_logger().info('1. Aproximacin a Punto A (Elevado)...')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a + z_lift_offset, request.roll_a, request.pitch_a, request.yaw_a):
            return self.fail(response, "Fallo en aproximacin A")

        # 2. Bajar a A
        self.get_logger().info('2. Bajando a Punto A...')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a, request.roll_a, request.pitch_a, request.yaw_a):
            return self.fail(response, "Fallo al bajar a A")

        # 3. Cerrar Garra
        self.get_logger().info('3. Cerrando Garra...')
        if not await self.send_gripper_command(0.7): # Ajusta este valor segn el objeto
            return self.fail(response, "Fallo al cerrar garra")
        time.sleep(1.0)

        # 4. Elevarse en Z
        self.get_logger().info('4. Elevando objeto...')
        if not await self.send_cartesian_goal(request.x_a, request.y_a, request.z_a + z_lift_offset, request.roll_a, request.pitch_a, request.yaw_a):
            return self.fail(response, "Fallo al elevar objeto")

        # 5. Posicionarse en B (Manteniendo altura)
        self.get_logger().info('5. Moviendo a Punto B (Manteniendo altura)...')
        if not await self.send_cartesian_goal(request.x_b, request.y_b, request.z_b + z_lift_offset, request.roll_b, request.pitch_b, request.yaw_b):
            return self.fail(response, "Fallo al mover a B elevado")

        # 6. Bajar a B
        self.get_logger().info('6. Bajando a Punto B...')
        if not await self.send_cartesian_goal(request.x_b, request.y_b, request.z_b, request.roll_b, request.pitch_b, request.yaw_b):
            return self.fail(response, "Fallo al bajar a B")

        # 7. Abrir Garra
        self.get_logger().info('7. Abriendo Garra...')
        if not await self.send_gripper_command(0.0):
            return self.fail(response, "Fallo al abrir garra")
        time.sleep(1.0)

        # 8. Retirarse (Home)
        self.get_logger().info('8. Retirando a Home...')
        await self.send_joint_goal([0.0]*6)
        
        response.success = True
        response.message = "Secuencia Pick and Place completada con exito."
        return response

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
