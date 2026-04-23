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
from kinova_apps.srv import SixMoves
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
import time

class SixMovesNode(Node):
    def __init__(self):
        super().__init__('six_moves_node')
        
        # Callback group for parallel execution (service + action client)
        self.group = ReentrantCallbackGroup()
        
        # Action Client for MoveIt 2
        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.group)
        
        # Service
        self.srv = self.create_service(
            SixMoves, 'six_moves', self.handle_service, callback_group=self.group)
        
        self.get_logger().info('Six Moves Node started.')
        
        # Timer for initial homing
        self.init_timer = self.create_timer(1.0, self.initial_homing, callback_group=self.group)

    async def initial_homing(self):
        self.init_timer.cancel()
        
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return
            
        self.get_logger().info('Homing: Moving all joints to zero...')
        await self.send_joint_goal([0.0]*6, speed=0.1)

    async def send_joint_goal(self, joint_values, speed=0.1):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = speed
        
        constraints = Constraints()
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = f'joint_{i+1}'
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_handle_future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = await goal_handle_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected!')
            return False
            
        result_wrapped = await goal_handle.get_result_async()
        return result_wrapped.status == GoalStatus.STATUS_SUCCEEDED

    async def send_cartesian_goal(self, x, y, z, roll, pitch, yaw, speed=0.1):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = speed

        # Position
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link'
        target_point = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01] # 1cm tolerance
        target_point.primitives.append(primitive)
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_point.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = target_point
        pos_constraint.weight = 1.0

        # Orientation
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'end_effector_link'
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rot.as_quat()
        ori_constraint.orientation.x = quat[0]
        ori_constraint.orientation.y = quat[1]
        ori_constraint.orientation.z = quat[2]
        ori_constraint.orientation.w = quat[3]
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(
            name="goal",
            position_constraints=[pos_constraint],
            orientation_constraints=[ori_constraint]
        ))

        goal_handle_future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = await goal_handle_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Cartesian goal rejected!')
            return False
            
        result_wrapped = await goal_handle.get_result_async()
        return result_wrapped.status == GoalStatus.STATUS_SUCCEEDED

    async def handle_service(self, request, response):
        num_coords = len(request.x)
        self.get_logger().info(f'Received service request with {num_coords} coordinates.')
        
        # We will process up to 6 coordinates as requested
        limit = min(num_coords, 6)
        
        for i in range(limit):
            x, y, z = request.x[i], request.y[i], request.z[i]
            roll, pitch, yaw = request.roll[i], request.pitch[i], request.yaw[i]
            
            self.get_logger().info(f'Moving to coordinate {i+1}/{limit}: x={x}, y={y}, z={z}')
            
            success = await self.send_cartesian_goal(x, y, z, roll, pitch, yaw)
            if not success:
                response.success = False
                response.message = f"Failed to reach coordinate index {i}."
                return response
            
            self.get_logger().info(f'Reached position {i+1}. Waiting 2 seconds...')
            time.sleep(2.0)
            
        self.get_logger().info('Sequence finished. Returning to zero position...')
        await self.send_joint_goal([0.0]*6, speed=0.1)
        
        response.success = True
        response.message = f"Successfully completed {limit} moves and returned home."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SixMovesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
