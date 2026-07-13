#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower_node')
        
        # Suscribirse al LiDAR y publicar al motor
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.ancho_carro = 0.5 # metros (burbuja de seguridad)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # 1. Limpiar datos inválidos (infinitos o nans)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)
        
        # 2. Filtrar solo los rangos de interés (la parte frontal de la pista)
        # Recortamos a +/- 90 grados (el carro no necesita mirar atrás)
        num_rayos = len(ranges)
        ventana = int(num_rayos / 4)
        rango_frontal = ranges[ventana : 3*ventana]
        
        # 3. Encontrar el punto más lejano (el centro de la brecha más grande)
        # En el FTG, buscamos el "hueco" (gap) más grande en las mediciones
        idx_max = np.argmax(rango_frontal)
        angulo_max = (idx_max - len(rango_frontal)/2) * msg.angle_increment
        
        # 4. Cálculo de dirección (Proporcional simple)
        # Si el hueco está a la derecha, girar a la derecha
        kp = 0.5
        steering_angle = angulo_max * kp
        
        # 5. Cálculo de velocidad (Frenar si hay paredes muy cerca)
        distancia_cercana = np.min(rango_frontal)
        velocidad = 2.0 if distancia_cercana > 1.0 else 0.5
        
        # 6. Publicar
        msg_drive = AckermannDriveStamped()
        msg_drive.drive.steering_angle = float(np.clip(steering_angle, -0.41, 0.41))
        msg_drive.drive.speed = float(velocidad)
        self.pub_drive.publish(msg_drive)

def main(args=None):
    rclpy.init(args=args)
    nodo = GapFollower()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
