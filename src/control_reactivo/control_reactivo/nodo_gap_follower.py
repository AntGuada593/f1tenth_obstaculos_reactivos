#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower_node')
        
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # --- Variables del Filtro Pasa-Bajas ---
        self.prev_steering = 0.0
        self.alpha = 0.35  
        
        # --- Variables del Cronómetro y Vuelta ---
        self.start_x = None
        self.start_y = None
        self.lap_count = 0
        self.last_lap_time = 0.0
        self.on_start_line = True
        self.has_started = False
        
        # --- Meta de la carrera ---
        self.target_laps = 10
        self.race_finished = False

    def odom_callback(self, msg):
        # Si la carrera ya terminó, no seguimos calculando la odometría
        if self.race_finished:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f"📍 Punto de meta registrado en: X={x:.2f}, Y={y:.2f}")
            self.get_logger().info(f"🏁 Meta configurada a {self.target_laps} vueltas.")
            return
        
        dist = np.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if not self.has_started:
            if dist > 0.5:
                self.has_started = True
                self.last_lap_time = current_time
                self.get_logger().info("🟢 ¡Cronómetro Iniciado! Vuelta 1 en curso...")
                
        if self.has_started:
            if dist > 8.0:
                self.on_start_line = False
            
            if dist < 2.0 and not self.on_start_line:
                self.lap_count += 1
                self.on_start_line = True
                lap_duration = current_time - self.last_lap_time
                self.last_lap_time = current_time
                
                self.get_logger().info(
                    f"\n====================================\n"
                    f"🏆 ¡VUELTA {self.lap_count} COMPLETADA!\n"
                    f"⏱️  Tiempo de vuelta: {lap_duration:.3f} segundos\n"
                    f"===================================="
                )
                
                # --- DETENER EL CARRO AL LLEGAR A LAS 10 VUELTAS ---
                if self.lap_count >= self.target_laps:
                    self.race_finished = True
                    self.get_logger().info(
                        f"\n🏁 ¡CARRERA COMPLETADA! 🏁\n"
                        f"Se han alcanzado las {self.target_laps} vueltas requeridas.\n"
                        f"Frenando el vehículo..."
                    )
                    
                    # Comando de freno total
                    stop_msg = AckermannDriveStamped()
                    stop_msg.drive.steering_angle = 0.0
                    stop_msg.drive.speed = 0.0
                    self.pub_drive.publish(stop_msg)

    def lidar_callback(self, msg):
        # Si la carrera ya terminó, el cerebro ignora el LiDAR y no manda acelerar
        if self.race_finished:
            return

        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)
        ranges = np.convolve(ranges, np.ones(5)/5, mode='same')
        
        num_rayos = len(ranges)
        inicio = int(num_rayos / 4)
        fin = int(3 * num_rayos / 4)
        rango_frontal = ranges[inicio:fin]
        
        idx_cercano = np.argmin(rango_frontal)
        burbuja = 10  
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
            steering_raw = np.clip(angle * 0.6, -0.41, 0.41)
            
            steering = (self.alpha * steering_raw) + ((1.0 - self.alpha) * self.prev_steering)
            self.prev_steering = steering
            
            velocidad_maxima = 6.5
            velocidad_minima = 4.0 
            
            penalizacion_giro = abs(steering) / 0.41 
            speed = velocidad_maxima - (velocidad_maxima - velocidad_minima) * penalizacion_giro
            
        else:
            target_idx_frontal = np.argmax(rango_frontal)
            target_idx_global = inicio + target_idx_frontal
            angle = (target_idx_global - num_rayos / 2) * msg.angle_increment
            steering_raw = np.clip(angle * 0.5, -0.41, 0.41)
            
            steering = (self.alpha * steering_raw) + ((1.0 - self.alpha) * self.prev_steering)
            self.prev_steering = steering
            speed = 1.0

        msg_drive = AckermannDriveStamped()
        msg_drive.drive.steering_angle = float(steering)
        msg_drive.drive.speed = float(speed)
        self.pub_drive.publish(msg_drive)

def main(args=None):
    rclpy.init(args=args)
    nodo = GapFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
