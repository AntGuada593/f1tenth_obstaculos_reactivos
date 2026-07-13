# 🏎️ F1TENTH: Control Reactivo y Evasión de Obstáculos (Follow The Gap)

**Autor:** Anthony Guadalupe Chávez  
**Descripción:** Este repositorio contiene la implementación completa de un sistema de navegación autónoma basado en el algoritmo *Follow The Gap* (FTG) para el simulador F1TENTH. Escrito en Python bajo el framework ROS 2, el proyecto escala desde la navegación en una pista despejada hasta la evasión de múltiples obstáculos estáticos y la superación de un obstáculo dinámico (vehículo oponente).

---

## 📑 Tabla de Contenidos
1. [Requisitos Previos del Sistema](#-requisitos-previos-del-sistema)
2. [Instalación del Simulador y Dependencias](#-instalación-del-simulador-y-dependencias)
3. [Instalación de este Repositorio](#-instalación-de-este-repositorio)
4. [Fase 1: FTG Inicial (Pista Despejada)](#-fase-1-ftg-inicial-pista-despejada)
5. [Fase 2: Obstáculos Fijos y Móviles (FTG Avanzado)](#-fase-2-obstáculos-fijos-y-móviles-ftg-avanzado)
6. [Demostración en Video](#-demostración-en-video)

---

## 💻 Requisitos Previos del Sistema

Para ejecutar este proyecto, tu sistema debe contar con lo siguiente:

* **Sistema Operativo:** Ubuntu 22.04 LTS.
* **Framework:** ROS 2 Humble Hawksbill. 
  *(Si no tienes ROS 2 instalado, sigue la [Guía oficial de instalación de ROS 2 Humble para Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) antes de continuar).*
* **Herramientas de compilación de ROS 2:**
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions
  
## 🛠️ Instalación del Simulador y Dependencias

El entorno virtual requiere el simulador oficial f1tenth_gym_ros. Abre una terminal y ejecuta los siguientes comandos para crear tu espacio de trabajo (workspace) y clonar el simulador:

  ```bash
  # 1. Crear el directorio del workspace
  mkdir -p ~/f1tenth_ws/src
  cd ~/f1tenth_ws/src

  # 2. Clonar el simulador oficial
  git clone [https://github.com/f1tenth/f1tenth_gym_ros.git](https://github.com/f1tenth/f1tenth_gym_ros.git)

  # 3. Compilar el simulador
  cd ~/f1tenth_ws
  colcon build --symlink-install

## 🚀 Instalación de este Repositorio
Ahora vamos a integrar los algoritmos de control y los mapas modificados en tu sistema.

  ```bash
  # 1. Navegar a la carpeta fuente
  cd ~/f1tenth_ws/src

  # 2. Clonar este repositorio (reemplaza TU_USUARIO por tu usuario de GitHub)
  git clone [https://github.com/TU_USUARIO/f1tenth_obstaculos_reactivos.git](https://github.com/TU_USUARIO/f1tenth_obstaculos_reactivos.git)

  # 3. Mover el paquete de control a la raíz de src para que ROS lo detecte
  mv f1tenth_obstaculos_reactivos/src/control_reactivo ~/f1tenth_ws/src/

  # 4. Compilar todo el workspace
  cd ~/f1tenth_ws
  colcon build --symlink-install
  source install/setup.bash

## 🟢 Fase 1: FTG Inicial (Pista Despejada)

Esta fase ejecuta la primera versión del algoritmo en el circuito original de Budapest.

  #Paso 1: Configurar el mapa y los vehículos
  Abre el archivo de configuración del simulador:

  ```bash
  gedit ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
  
  Asegúrate de que la configuración tenga 1 solo agente y apunte al mapa original:

  num_agent: 1
  map_path: 'Budapest_map'
  
  Guarda y cierra el archivo. Reemplaza los archivos del mapa original si es necesario:

  ``bash
  cp ~/f1tenth_ws/src/control_reactivo/mapas/Budapest_original/* ~/f1tenth_ws/src/f1tenth_gym_ros/maps/
  Paso 2: Ejecución
  Abre dos terminales distintas.

  Terminal 1 (Lanzar el Simulador):

  ```bash
  cd ~/f1tenth_ws
  source install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  
  Terminal 2 (Lanzar el algoritmo inicial):

  ```bash
  cd ~/f1tenth_ws
  source install/setup.bash
  ros2 run control_reactivo ftg_inicial

## 🔴 Fase 2: Obstáculos Fijos y Móviles (FTG Avanzado)

En esta fase, introducimos un mapa modificado con 5 obstáculos fijos y un segundo vehículo actuando como obstáculo dinámico. El algoritmo principal ha sido mejorado con un Filtro Pasa-Bajas y un sistema de telemetría que frena el vehículo automáticamente tras completar 10 vueltas.

  #Paso 1: Cargar el mapa con obstáculos
  
  Copia el mapa modificado al directorio del simulador:

  ```bash
  cp ~/f1tenth_ws/src/control_reactivo/mapas/Budapest_modificado/* ~/f1tenth_ws/src/f1tenth_gym_ros/maps/
  
  #Paso 2: Configurar el modo Multi-Agente
  
  Edita nuevamente el archivo de configuración:

  ```bash
  gedit ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
  
  Modifica las variables para declarar 2 agentes y establecer sus coordenadas exactas de salida:

  num_agent: 2
  poses_x: [92.8, 94.8]
  poses_y: [110.8, 110.8]
  poses_theta: [0.0, 0.0]
  
  Guarda y cierra el archivo.

  #Paso 3: Ejecución Síncrona
  
  Abre tres terminales distintas. El simulador de ROS 2 en modo multi-agente congelará el tiempo hasta que todos los vehículos reciban comandos.

  Terminal 1 (El Simulador):

  ´´´bash
  cd ~/f1tenth_ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  
  Terminal 2 (Auto Principal - Cerebro FTG Avanzado):

  ```bash
  cd ~/f1tenth_ws
  source install/setup.bash
  ros2 run control_reactivo ejecutable_gap
  
  (El vehículo principal se conectará, pero esperará a que el oponente se despierte).

  Terminal 3 (Auto Oponente - Cerebro Zombie):

  ```bash
  cd ~/f1tenth_ws
  source install/setup.bash
  ros2 run control_reactivo ejecutable_zombie
  
  (Al presionar Enter en esta terminal, comenzará la carrera).

## 🎥 Demostración en Video
A continuación se observa al algoritmo esquivando los obstáculos fijos y rebasando exitosamente al vehículo dinámico mediante cálculos reactivos del LiDAR.

(El archivo de video original se encuentra disponible en la carpeta media/ de este repositorio).
