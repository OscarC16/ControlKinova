#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from kinova_apps.srv import MoveAndCapture

class TestServiceClient(Node):
    def __init__(self):
        super().__init__('test_service_client')
        self.cli = self.create_client(MoveAndCapture, 'move_and_capture')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')
        self.req = MoveAndCapture.Request()

    def send_request(self, x, y, z, roll, pitch, yaw):
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.z = float(z)
        self.req.roll = float(roll)
        self.req.pitch = float(pitch)
        self.req.yaw = float(yaw)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    client = TestServiceClient()
    
    # Coordenadas de prueba (puedes ajustarlas)
    x, y, z = 0.45, 0.0, 0.2
    roll, pitch, yaw = 180.0, 0.0, 90.0
    
    print(f"Llamando al servicio con: x={x}, y={y}, z={z}")
    response = client.send_request(x, y, z, roll, pitch, yaw)
    
    if response:
        print(f"Resultado: {response.success}")
        print(f"Mensaje: {response.message}")
        if response.image.data:
            print("Imagen recibida correctamente.")
    else:
        print("Error al llamar al servicio.")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
