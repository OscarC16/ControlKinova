#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kinova_apps.srv import SixMoves

class TestSixMoves(Node):
    def __init__(self):
        super().__init__('test_six_moves')
        self.cli = self.create_client(SixMoves, 'six_moves')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "six_moves" no disponible, esperando...')
        self.req = SixMoves.Request()

    def send_request(self):
        # Ejemplo de 6 coordenadas cartesianas
        # Ajusta estos valores segn el espacio de trabajo de tu robot
        self.req.x = [0.4, 0.45, 0.4, 0.35, 0.3, 0.35]
        self.req.y = [0.0, 0.1, 0.15, 0.0, -0.1, -0.15]
        self.req.z = [0.4, 0.4, 0.45, 0.5, 0.45, 0.4]
        
        # Orientacin constante (mirando hacia abajo)
        self.req.roll = [180.0] * 6
        self.req.pitch = [0.0] * 6
        self.req.yaw = [90.0] * 6
        
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    test_node = TestSixMoves()
    test_node.get_logger().info('Enviando solicitud de 6 movimientos...')
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
