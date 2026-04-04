import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation as R
import sys

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('moveit_action_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self, x, y, z, roll, pitch, yaw):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Constraint for target position
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link' # Need to verify this link name

        target_point = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]
        target_point.primitives.append(primitive)
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_point.primitive_poses.append(target_pose.pose)
        
        pos_constraint.constraint_region = target_point
        pos_constraint.weight = 1.0

        # Constraint for target orientation
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'end_effector_link'
        
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rot.as_quat()
        ori_constraint.orientation.x = quat[0]
        ori_constraint.orientation.y = quat[1]
        ori_constraint.orientation.z = quat[2]
        ori_constraint.orientation.w = quat[3]
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(
            name="goal",
            position_constraints=[pos_constraint],
            orientation_constraints=[ori_constraint]
        ))

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={z}')
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = MoveItActionClient()
    
    # Example Target
    future = client.send_goal(0.3, 0.3, 0.3, 0.0, 0.0, 0.0)
    rclpy.spin_until_future_complete(client, future)
    
    client.get_logger().info('Goal sent, check MoveIt console/RViz')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
