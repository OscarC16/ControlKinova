#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kinova_apps.srv import PickAndPlace

class TestPickPlace(Node):
    def __init__(self):
        super().__init__('test_pick_place')
        self.cli = self.create_client(PickAndPlace, 'pick_and_place')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio "pick_and_place" no disponible, esperando...')
        self.req = PickAndPlace.Request()

    def send_request(self):
        # Punto A (Pick - Donde esta el objeto)
        self.req.x_a = 0.4
        self.req.y_a = 0.4 # Movido ligeramente a un lado
        self.req.z_a = 0.4
        self.req.roll_a = 180.0
        self.req.pitch_a = 0.0
        self.req.yaw_a = 90.0
        
        # Punto B (Place - Donde dejar el objeto)
        self.req.x_b = -0.4
        self.req.y_b = -0.4 # Movido al lado opuesto
        self.req.z_b = 0.4
        self.req.roll_b = 180.0
        self.req.pitch_b = 0.0
        self.req.yaw_b = 90.0
        
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    test_node = TestPickPlace()
    test_node.get_logger().info('Enviando solicitud Pick and Place de prueba...')
    future = test_node.send_request()
    
    rclpy.spin_until_future_complete(test_node, future)
    
    result = future.result()
    if result:
        if result.success:
            test_node.get_logger().info(f'XITO: {result.message}')
        else:
            test_node.get_logger().error(f'FALLO: {result.message}')
    else:
        test_node.get_logger().error('Error al llamar al servicio.')
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
