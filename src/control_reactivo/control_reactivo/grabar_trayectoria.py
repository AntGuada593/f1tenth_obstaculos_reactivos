#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math

class GrabadorTrayectoria(Node):
    def __init__(self):
        super().__init__('grabador_trayectoria')
        
        # Nos suscribimos a la posición del carro
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        # Abrimos un archivo CSV para escribir los puntos
        self.archivo = open('trayectoria_budapest.csv', 'w', newline='', buffering=1)
        self.escritor = csv.writer(self.archivo)
        self.escritor.writerow(['x', 'y'])  # Cabeceras
        
        self.ultimo_x = 0.0
        self.ultimo_y = 0.0
        self.es_primer_punto = True
        
        self.get_logger().info('Grabadora de Puntos INICIADA. Haz que el carro dé una vuelta...')

    def odom_callback(self, msg):
        # Extraemos la coordenada global del carro
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.es_primer_punto:
            self.escritor.writerow([x, y])
            self.ultimo_x = x
            self.ultimo_y = y
            self.es_primer_punto = False
            return
            
        # Solo guardamos el punto si el carro avanzó al menos 10 centímetros
        # (Para no saturar el archivo si el carro está quieto)
        distancia = math.hypot(x - self.ultimo_x, y - self.ultimo_y)
        if distancia > 0.1:
            self.escritor.writerow([x, y])
            self.ultimo_x = x
            self.ultimo_y = y

    def destroy_node(self):
        # Asegurarnos de cerrar y guardar el archivo al apagar el nodo
        self.archivo.close()
        self.get_logger().info('Archivo trayectoria_budapest.csv guardado con éxito.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo = GrabadorTrayectoria()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
