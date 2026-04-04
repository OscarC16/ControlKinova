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
from sensor_msgs.msg import Image
from kinova_apps.srv import MoveAndCapture
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import cv2
import asyncio
import time
import os

class RobotServiceNode(Node):
    def __init__(self):
        super().__init__('robot_service_node')
        self.declare_parameter('robot_ip', '192.168.1.10')
        self.robot_ip = self.get_parameter('robot_ip').value
        self.bridge = CvBridge()
        
        # Callback group for parallel execution
        self.group = ReentrantCallbackGroup()
        
        # Action Client for MoveIt 2
        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.group)
        
        
        # Service
        self.srv = self.create_service(
            MoveAndCapture, 'move_and_capture', self.handle_service, callback_group=self.group)
        
        # Suscriptores para temas de imagen (como fallback)
        self.latest_img_msg = None
        self.create_subscription(Image, '/camera/color/image_raw', self.img_callback, 10, callback_group=self.group)
        self.create_subscription(Image, '/kinova_vision/color/image_raw', self.img_callback, 10, callback_group=self.group)
        
        self.get_logger().info('Robot Service Node started.')
        self.get_logger().info('Waiting for MoveGroup action server...')
        
        # Timer to start homing after node is ready
        self.init_timer = self.create_timer(1.0, self.initial_homing, callback_group=self.group)

    def img_callback(self, msg):
        self.latest_img_msg = msg

    async def initial_homing(self):
        # Cancel timer so it only runs once
        self.init_timer.cancel()
        
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return
            
        self.get_logger().info('Starting initial homing (all joints to zero)...')
        await self.send_joint_goal([0.0]*6, speed=0.1)

    async def send_joint_goal(self, joint_values, speed=0.15, accel=0.05):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = accel
        
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
        
        # Using await for action calls
        # Note: wait_for_server call is handled in initial_homing or service call
        
        goal_handle_future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = await goal_handle_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected!')
            return False
            
        result_wrapped = await goal_handle.get_result_async()
        if result_wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Joint goal reached successfully.')
            return True
        else:
            self.get_logger().error(f'Joint goal failed with status: {result_wrapped.status}')
            return False

    async def send_cartesian_goal(self, x, y, z, roll, pitch, yaw, speed=0.15, accel=0.05):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = accel

        # Position
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link'
        target_point = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01] # Tolerancia de posicin: 1cm
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

        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server timed out.')
            return False

        goal_handle_future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = await goal_handle_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Cartesian goal rejected!')
            return False
            
        result_wrapped = await goal_handle.get_result_async()
        if result_wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Cartesian goal reached successfully.')
            return True
        else:
            self.get_logger().error(f'Cartesian goal failed with status: {result_wrapped.status}')
            return False

    def capture_image_threaded(self):
        """Intenta capturar imagen en un hilo separado con timeout para evitar bloqueos."""
        import threading
        
        result = {"frame": None, "error": "Timeout"}
        
        def target():
            try:
                # Intentar con y sin credenciales
                options = 'rtsp_transport;tcp|fflags;nobuffer'
                os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = options
                
                # Probar URLs comunes
                urls = [
                    f"rtsp://{self.robot_ip}/color",
                    f"rtsp://admin:admin@{self.robot_ip}/color"
                ]
                
                for url in urls:
                    self.get_logger().info(f'Probando URL: {url}')
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                    if cap.isOpened():
                        # Esperar un momento a que el stream se inicie
                        for _ in range(5): cap.grab()
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            result["frame"] = frame
                            result["error"] = None
                            cap.release()
                            return # Exito
                        cap.release()
                
                result["error"] = "No se pudo abrir ningn stream (probado con y sin admin:admin)"
            except Exception as e:
                result["error"] = str(e)

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()
        thread.join(timeout=10.0) # Timeout de 10 segundos para la cmara
        
        if result["error"]:
            self.get_logger().error(f'Error en captura: {result["error"]}')
            return None
        return result["frame"]

    def capture_image(self):
        self.get_logger().info('--- Iniciando Captura de Imagen ---')
        frame = self.capture_image_threaded()
        
        # Si fall la captura por RTSP, intentar usar el ltimo mensaje recibido por tpico ROS
        if frame is None and self.latest_img_msg is not None:
            self.get_logger().info('RTSP fall, usando imagen del tpico ROS...')
            try:
                frame = self.bridge.imgmsg_to_cv2(self.latest_img_msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f'Error al convertir imagen de tpico: {e}')
        
        if frame is not None:
            # Guardar localmente
            save_dir = "/mnt/d/Ros/captures"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"capture_{timestamp}.png"
            filepath = os.path.join(save_dir, filename)
            
            cv2.imwrite(filepath, frame)
            self.get_logger().info(f'Imagen guardada: {filepath}')
            
            return self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        self.get_logger().error('No se pudo obtener imagen ni por RTSP ni por tpico ROS.')
        return None

    async def handle_service(self, request, response):
        self.get_logger().info(f'Received service call: x={request.x}, y={request.y}, z={request.z}')
        
        response.image = Image()
        
        # 1. Move to Cartesian Position
        self.get_logger().info('Moving to target...')
        if not await self.send_cartesian_goal(request.x, request.y, request.z, request.roll, request.pitch, request.yaw, speed=0.15, accel=0.05):
            response.success = False
            response.message = "Failed to move to target."
            return response
            
        # 2. Wait a few seconds
        self.get_logger().info('Esperando 3 segundos para que la imagen se estabilice...')
        time.sleep(3.0)
        
        # 3. Capture Image
        self.get_logger().info('Capturing image...')
        img_msg = self.capture_image()
        if img_msg:
            response.image = img_msg
            self.get_logger().info('Image captured and added to response.')
        else:
            self.get_logger().warn('Failed to capture image.')
            
        # 4. Return to Home
        self.get_logger().info('Returning to home...')
        if not await self.send_joint_goal([0.0]*6, speed=0.15, accel=0.05):
            response.success = False
            response.message = "Moved to target and captured image, but failed to return to home."
            return response
            
        response.success = True
        response.message = "Service completed successfully. Image returned in response."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotServiceNode()
    
    # Use MultiThreadedExecutor to allow callbacks to run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
