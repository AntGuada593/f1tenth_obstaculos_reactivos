#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ZombieGapFollower(Node):
    def __init__(self):
        super().__init__('zombie_gap_follower_node')
        # ¡ATENCIÓN A LOS TÓPICOS! Se conectan al segundo auto
        self.sub_lidar = self.create_subscription(LaserScan, '/opp_scan', self.lidar_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)
        ranges = np.convolve(ranges, np.ones(5)/5, mode='same')
        
        num_rayos = len(ranges)
        inicio = int(num_rayos / 4)
        fin = int(3 * num_rayos / 4)
        rango_frontal = ranges[inicio:fin]
        
        idx_cercano = np.argmin(rango_frontal)
        burbuja = 20
        idx_start_burbuja = max(0, idx_cercano - burbuja)
        idx_end_burbuja = min(len(rango_frontal), idx_cercano + burbuja)
        rango_frontal[idx_start_burbuja:idx_end_burbuja] = 0.0
        
        is_gap = (rango_frontal > 1.5).astype(int)
        is_gap_padded = np.pad(is_gap, 1, mode='constant', constant_values=0)
        d = np.diff(is_gap_padded)
        starts = np.where(d == 1)[0]
        ends = np.where(d == -1)[0]
        
        if len(starts) > 0:
            widths = ends - starts
            best_idx = np.argmax(widths)
            
            target_idx_padded = int((starts[best_idx] + ends[best_idx]) / 2)
            target_idx_frontal = target_idx_padded - 1
            target_idx_global = inicio + target_idx_frontal
            
            angle = (target_idx_global - num_rayos / 2) * msg.angle_increment
            steering = np.clip(angle * 0.6, -0.41, 0.41)
            
            # VELOCIDAD ZOMBIE (Lenta y constante para ser un obstáculo móvil)
            velocidad_maxima = 2.0
            velocidad_minima = 1.5 
            
            penalizacion_giro = abs(steering) / 0.41 
            speed = velocidad_maxima - (velocidad_maxima - velocidad_minima) * penalizacion_giro
        else:
            target_idx_frontal = np.argmax(rango_frontal)
            target_idx_global = inicio + target_idx_frontal
            angle = (target_idx_global - num_rayos / 2) * msg.angle_increment
            steering = np.clip(angle * 0.5, -0.41, 0.41)
            speed = 0.5

        msg_drive = AckermannDriveStamped()
        msg_drive.drive.steering_angle = float(steering)
        msg_drive.drive.speed = float(speed)
        self.pub_drive.publish(msg_drive)

def main(args=None):
    rclpy.init(args=args)
    nodo = ZombieGapFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
