<div align="center">

# Axioma.io   :hugs: :muscle: :seedling: :nerd_face:

### Robot Móvil Autónomo con SLAM y Navegación Nav2

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-yellow?logo=gazebo&logoColor=white)](http://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Enabled-brightgreen)](https://navigation.ros.org/)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-red)](https://github.com/SteveMacenski/slam_toolbox)

</div>

---

## 🚀 Quick Start

```bash
# 1. Instalar ROS2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install ros-humble-desktop

# 2. Instalar TODAS las dependencias (un solo comando)
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
                     gazebo ros-humble-gazebo-ros-pkgs \
                     ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
                     ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
                     ros-humble-joy ros-humble-joy-linux ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
                     ros-humble-tf2-tools ros-humble-tf-transformations \
                     ros-humble-rviz2 \
                     ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers \
                     ros-humble-rqt-robot-steering

# 3. Clonar y compilar el proyecto
mkdir -p ~/ros2/axioma_humble_ws/src
cd ~/ros2/axioma_humble_ws/src
git clone https://github.com/mrdavidavl/Axioma_robot.git .
cd ~/ros2/axioma_humble_ws
colcon build
source install/setup.bash

# 4. Lanzar navegación autónoma (requiere mapa previo)
ros2 launch axioma_bringup navigation_bringup.launch.py

# O crear un mapa nuevo con SLAM
ros2 launch axioma_bringup slam_bringup.launch.py
```

**📖 Ver [Instalación Completa](#instalación-y-configuración) | [Guía de Uso](#🚀-los-3-launches-esenciales)**

---

## 📖 Descripción

Este proyecto de grado aborda el desarrollo de software con el sistema operativo de robots ROS2 para convertir la plataforma robótica móvil Axioma.io desarrollado por estudiantes del semillero de robótica SIRO en una plataforma autónoma con la capacidad de percibir y entender el entorno de trabajo en el que se encuentre y pueda calcular una ruta para desplazarse de un punto de origen a un punto final llevando a bordo algún producto, todo esto sin intervención de un operario y sirviendo así como una solución a la automatización de la logística en cadenas y/o procesos de producción cumpliendo los requerimientos de la industria en el suministro, planificación , gestión y control del almacenamiento de mercancía para conseguir los niveles más altos de servicio, calidad y eficiencia al menor tiempo y costo posible.

### 🔑 Palabras clave

Robot móvil, autónomo, logística, planeación, trayectorias, ROS2 Humble, Nav2, SLAM

#### Objetivo general

*Diseñar, simular e implementar software de planificación de trayectorias robóticas para la plataforma de robótica móvil Axioma.io con el fin de cumplir con la función de transportar productos de forma autónoma desde un punto inicial a un punto final dentro de un espacio de trabajo determinado, garantizando la automatización de la logística en la gestión y coordinación de estas actividades dentro de una cadena o proceso de producción.*

##### Objetivos específicos

* Diseñar un entorno de trabajo tridimensional que simula el área de trabajo con obstáculos.
* Instrumentar el robot virtual con los sensores encargados de orientación, posición, navegación y mapeo.
* Programar el ecosistema de ROS con todos paquetes de los nodos necesarios para la localización, control y navegación, así como también el mapeo del entorno de trabajo del robot.
* Desarrollar las técnicas de planeación de trayectorias robóticas que indique el camino para que el robot pueda ir de un punto a otro.
* Integrar el software desarrollado para la localización, control, navegación, mapeo y planificación de trayectorias  en el robot físico Axioma.io.
Agregar sensores para realizar odometría y cálculo de velocidad de giros del robot.

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/axioma.jpeg" alt="Axioma Robot" width="400">
</div>

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/open_software.jpeg" alt="Open Source" width="400">
</div>

---

## 🌟 Características Principales

<div align="center">

| Feature | Descripción |
|---------|-------------|
| 🗺️ **SLAM en Tiempo Real** | Mapeo simultáneo y localización con SLAM Toolbox |
| 🎯 **Navegación Autónoma** | Sistema Nav2 completo con planificación de rutas |
| 🚧 **Evitación de Obstáculos** | Detección y evasión en tiempo real con LIDAR 360° |
| 🎮 **Control Manual** | Soporte para joystick Xbox durante mapeo |
| 📊 **Visualización Completa** | RViz2 con costmaps, trayectorias y partículas AMCL |
| 🤖 **Robot Diferencial** | Odometría precisa con encoders 1000 PPR |
| 🔧 **Totalmente Configurable** | Parámetros Nav2, AMCL y SLAM ajustables |
| 💻 **Código Abierto** | Licencia BSD - Libre para uso académico y comercial |

</div>

---

## 🎥 Demostración del Proyecto

> **Nota:** Los videos mostrados corresponden a la versión con ROS2 Foxy. La funcionalidad en Humble es idéntica con mejoras en rendimiento y estabilidad.

### Videos del Sistema Funcionando

<div align="center">

| **Navegación Autónoma** | **SLAM y Mapeo** |
|:------------------------:|:-----------------:|
| [![Axioma Navigation Part 1](https://img.youtube.com/vi/U28n4vSAwDk/0.jpg)](https://youtu.be/U28n4vSAwDk) | [![Axioma SLAM Part 2](https://img.youtube.com/vi/A-7UMoYXUBQ/0.jpg)](https://youtu.be/A-7UMoYXUBQ) |
| *Navegación en entorno con mapa previamente cargado* | *Robot navegando con LIDAR creando mapa en tiempo real* |

| **Sensores y Frames** | **Ensamblaje 3D** |
|:---------------------:|:-----------------:|
| [![Axioma Sensors Part 3](https://img.youtube.com/vi/dHnnpMOO5yg/0.jpg)](https://youtu.be/dHnnpMOO5yg) | [![Axioma Assembly](https://img.youtube.com/vi/buS84GiqQug/0.jpg)](https://youtu.be/buS84GiqQug) |
| *Robot en movimiento, sensores en RViz y frames* | *Ensamblaje del robot en Autodesk Inventor* |

| **Concurso Mercury Robotics** | **Plataforma Teleoperada** |
|:-----------------------------:|:--------------------------:|
| [![Mercury Challenge 2019](https://img.youtube.com/vi/8E0mYynNUog/0.jpg)](https://youtu.be/8E0mYynNUog) | [![Axioma Teleop](https://img.youtube.com/vi/sHgdL3dffgw/0.jpg)](https://youtu.be/sHgdL3dffgw) |
| *Axioma One en Mercury Robotics Challenge 2019* | *Robot teleoperado con Raspberry Pi y Flask* |

</div>

### 🏆 Características Destacadas en los Videos

- **Navegación Autónoma**: Planificación de rutas y evitación de obstáculos
- **SLAM en Tiempo Real**: Mapeo simultáneo y localización con LIDAR
- **Visualización en RViz**: Monitoreo de sensores y transformadas
- **Diseño Mecánico**: Estructura robusta para aplicaciones industriales
- **Control Remoto**: Interfaz web para teleoperación

---

## 📋 Tabla de Contenidos

- [🚀 Quick Start](#🚀-quick-start)
- [🌟 Características Principales](#🌟-características-principales)
- [🎥 Demostración del Proyecto](#🎥-demostración-del-proyecto)
- [💻 Hardware](#1-hardware)
  - [Sensores y Actuadores](#sensores-y-actuadores)
  - [Micros (Arduino + Raspberry Pi)](#micros)
  - [Diseño 3D](#diseño-y-modelado-3d)
- [🔧 Software](#2-software)
  - [ROS2 Humble](#ros2-humble)
  - [Arquitectura](#arquitectura-ros2)
  - [Conceptos Fundamentales](#conceptos-fundamentales-de-ros2)
- [📐 Modelo Matemático](#4-modelo-matemático)
- [⚙️ Instalación y Configuración](#instalación-y-configuración)
- [🚀 Los 3 Launches Esenciales](#🚀-los-3-launches-esenciales)
  - [SLAM (Mapeo)](#1️⃣-slam-mapping---crear-mapas-nuevos)
  - [Guardar Mapa](#2️⃣-guardar-mapa---exportar-mapa-creado)
  - [Navegación Autónoma](#3️⃣-navegación-autónoma---usar-mapa-guardado)
- [🐛 Solución de Problemas](#🐛-solución-de-problemas)
- [🔧 Monitoreo y Depuración](#🔧-monitoreo-y-depuración)
- [💻 Desarrollo con ROS2](#🚀-desarrollo-con-ros2)
- [🤝 Contribuir](#🤝-contribuir)
- [📝 Licencia](#📝-licencia)

## 1 Hardware

El hardware se encuentra compuesto por dos herramientas open hardware muy usadas en el desarrollo y prototipado rapido de dispositivos electrónicos y mecátronicos y una raspberry pi. Estos dispositivos estan clasificados ***"One Chip"*** por todo en uno solo como lo es arduino que posee un microcontrolador, chip para la comunicación serial, reguladores de voltajes y demas componentes electronicos que permitan conectar actuadores y sensores de forma facil y rapida. En la clasificación One Chip tambien tenemos lo que es la raspberry pi que lleva a bordo un chip microprocesador, ram, video, ethernet/wifi, regulador, comunicación serial que le permiten conectar otros dispositivos como camaras, monitores y toda clase de perifericos usb que le brindan a esta pequeña tarjeta la posibilidad de crear muchas aplicaciones web, IoT, Entretenimiento y Robotica.

### Sensores y actuadores

#### LIDAR
El sensor LIDAR (Light Detection and Ranging) es fundamental para la navegación autónoma del robot Axioma.io. Este sensor utiliza pulsos de luz láser para medir distancias y crear un mapa 2D del entorno.

**Características técnicas:**
- Rango de detección: 0.1m - 10m
- Resolución angular: 1°
- Frecuencia de escaneo: 10 Hz
- Interfaz: USB/Serial
- Campo de visión: 360°

**Aplicaciones en Axioma:**
- Detección de obstáculos
- Mapeo SLAM
- Localización
- Navegación autónoma

#### Cámara
Sistema de visión por computadora para reconocimiento de objetos y navegación visual.

**Especificaciones:**
- Resolución: 640x480 px
- Frame rate: 30 fps
- Interfaz: USB 2.0
- Formato: RGB/BGR
- Ángulo de visión: 60°

**Funcionalidades:**
- Reconocimiento de objetos
- Seguimiento de líneas
- Detección de marcadores ArUco
- Navegación visual

#### Encoders
Sensores de posición rotativa instalados en cada rueda para odometría precisa.

**Características:**
- Tipo: Encoder incremental óptico
- Resolución: 1000 PPR (Pulsos Por Revolución)
- Salida: Cuadratura (A/B)
- Voltaje de operación: 5V
- Frecuencia máxima: 200 kHz

**Datos proporcionados:**
- Posición angular de las ruedas
- Velocidad de rotación
- Dirección de giro
- Distancia recorrida

#### Motores DC
Sistema de tracción diferencial con dos motores DC con reductores.

**Especificaciones técnicas:**
- Voltaje nominal: 12V DC
- Corriente nominal: 2A
- Velocidad sin carga: 180 RPM
- Torque nominal: 5 kg⋅cm
- Relación de reducción: 1:30
- Eficiencia: 85%

### Fuente de alimentación

El sistema de alimentación está diseñado para proporcionar energía estable y confiable a todos los componentes del robot.

**Configuración del sistema:**
- **Batería principal**: Li-Po 3S (11.1V nominal, 12.6V máximo)
- **Capacidad**: 5000 mAh
- **Reguladores de voltaje**:
  - 12V → 5V (5A) para Raspberry Pi y sensores
  - 12V → 3.3V (2A) para Arduino y periféricos
- **Protecciones**: Fusibles, circuitos de protección contra sobrecorriente
- **Autonomía estimada**: 4-6 horas de operación continua

**Distribución de corriente:**
| Componente | Voltaje | Corriente | Potencia |
|------------|---------|-----------|----------|
| Raspberry Pi 4 | 5V | 1.5A | 7.5W |
| Arduino Mega | 5V | 0.5A | 2.5W |
| Motores DC (x2) | 12V | 4A | 48W |
| LIDAR | 5V | 0.8A | 4W |
| Cámara | 5V | 0.3A | 1.5W |
| **Total** | - | **7.1A** | **63.5W** |

### Micros

#### [Arduino](https://www.arduino.cc/)

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/arduino.jpeg" alt="Arduino" width="350">
</div>

**Arduino Mega 2560** actúa como unidad de control de bajo nivel, encargándose de:

**Funciones principales:**
- Control PWM de motores DC
- Lectura de encoders con interrupciones
- Comunicación serial con Raspberry Pi
- Control de actuadores auxiliares
- Monitoreo de sensores analógicos

**Especificaciones:**
- Microcontrolador: ATmega2560
- Voltaje de operación: 5V
- Pines digitales: 54 (15 PWM)
- Pines analógicos: 16
- Memoria Flash: 256 KB
- SRAM: 8 KB
- EEPROM: 4 KB
- Frecuencia de reloj: 16 MHz

**Código ejemplo para control de motores:**
```cpp
// Control de motores con PWM
#define MOTOR_L_PWM 5
#define MOTOR_R_PWM 6
#define MOTOR_L_DIR 7
#define MOTOR_R_DIR 8

void setMotorSpeeds(int left_speed, int right_speed) {
    digitalWrite(MOTOR_L_DIR, left_speed > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_DIR, right_speed > 0 ? HIGH : LOW);
    analogWrite(MOTOR_L_PWM, abs(left_speed));
    analogWrite(MOTOR_R_PWM, abs(right_speed));
}
```

#### [Raspberry Pi](https://www.raspberrypi.com/)

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/raspberry.jpeg" alt="Raspberry Pi" width="350">
</div>

**Raspberry Pi 4 Model B** funciona como cerebro principal del robot, ejecutando ROS2 y algoritmos de alto nivel.

**Especificaciones:**
- SoC: Broadcom BCM2711 (Cortex-A72 64-bit)
- CPU: 4 núcleos a 1.5GHz
- RAM: 4GB LPDDR4
- Conectividad: WiFi 802.11ac, Bluetooth 5.0, Ethernet Gigabit
- USB: 4 puertos USB 3.0/2.0
- GPIO: 40 pines
- Almacenamiento: MicroSD 64GB

**Responsabilidades principales:**
- Ejecución del stack completo de ROS2
- Procesamiento de imágenes y LIDAR
- Algoritmos de navegación y SLAM
- Comunicación inalámbrica
- Interfaz web de control
- Logging y telemetría

### Diseño y Modelado 3D

El robot Axioma.io cuenta con un diseño mecánico completo desarrollado en software CAD profesional, permitiendo la fabricación y prototipado de todos sus componentes.

#### Modelo 3D Interactivo

<div align="center">
  <a href="https://www.autodesk.com/community/gallery/project/147581/robot-axioma-io-with-raspberry-pi-and-python">
    <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/axioma.jpeg" alt="Modelo 3D Axioma.io" width="400">
  </a>
  <br>
  <strong><a href="https://www.autodesk.com/community/gallery/project/147581/robot-axioma-io-with-raspberry-pi-and-python">🔗 Ver Modelo 3D Interactivo en Autodesk Gallery</a></strong>
</div>

#### Características del Diseño

- **Plataforma modular** para fácil ensamblaje y mantenimiento
- **Estructura robusta** optimizada para aplicaciones industriales
- **Compartimentos específicos** para Raspberry Pi, Arduino y sensores
- **Sistema de montaje** para LIDAR y cámara
- **Chasis diferencial** para tracción de dos ruedas
- **Materiales**: Aluminio, PLA/ABS para impresión 3D
- **Herramientas CAD**: Autodesk Inventor, Fusion 360

#### Archivos de Diseño

| Componente | Descripción | Formato |
|------------|-------------|---------|
| **Chasis Principal** | Estructura base del robot | `.ipt`, `.stl` |
| **Soportes de Sensores** | Montajes para LIDAR y cámara | `.ipt`, `.stl` |
| **Carcasa Electrónica** | Protección para PCBs | `.ipt`, `.stl` |
| **Sistema de Tracción** | Acoples para motores y ruedas | `.ipt`, `.stl` |
| **Ensamble Completo** | Modelo integrado | `.iam`, `.step` |

💡 **Nota**: Los archivos CAD están disponibles para modificación y mejora por parte de la comunidad maker.

## 2 Software

### 2.1 [ROS/ROS2](https://www.ros.org/)

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/ros.jpeg" alt="ROS2" width="400">
</div>

[source](https://github.com/ros)

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/service.gif" alt="ROS2 Service" width="400">
</div>

### [ROS2 Humble](https://docs.ros.org/en/humble/index.html)

**ROS2** (*Robot Operating System 2*) es una plataforma de código abierto diseñada para facilitar el desarrollo, operación y mantenimiento de sistemas robóticos y de automatización industrial. Ofrece una arquitectura modular y flexible que permite la comunicación entre componentes distribuidos, soportando una variedad de sistemas operativos y arquitecturas de hardware. ROS 2 se destaca por su capacidad de escalabilidad, seguridad y robustez, lo que lo convierte en una herramienta crucial para la creación de sistemas robóticos avanzados en diversos entornos industriales y de investigación.

#### Historia de ROS2

**ROS** en su primera versión, **ROS1**, se desarrolló en los Laboratorios de Inteligencia Artificial de Stanford (SAIL) por estudiantes de doctorado **Eric Berger** y **Keenan Wyrobek**. Se publicó bajo una **licencia BSD** de software libre en 2007, que permite libertad para uso comercial e investigador. Desde 2008, el instituto **Willow Garage** se ha encargado principalmente del desarrollo y soporte.

La idea de crear un sistema operativo era estandarizar tareas como la *abstracción de hardware*, *control de dispositivos* de bajo nivel (drivers), implementación de *procesos comunes*, manejo de *comunicación*, *soporte* de paquetes y otras ventajas.

**ROS2** es la evolución natural del exitoso marco de trabajo **ROS1**. Desarrollado para abordar las limitaciones de su predecesor, ROS2 ofrece una *arquitectura modular* y *distribuida*, mejor *rendimiento* y *escalabilidad*, así como soporte *multiplataforma*. Lanzado oficialmente en 2015, ROS2 mantiene la *flexibilidad* y *robustez* de ROS1, al tiempo que introduce mejoras significativas en herramientas de desarrollo y comunicación. Su diseño modular permite una fácil integración con otros sistemas y una adaptación más rápida a diferentes entornos de desarrollo. Con características como compatibilidad con múltiples lenguajes de programación y una creciente comunidad de desarrolladores, ROS2 es la elección preferida para proyectos de robótica modernos y ambiciosos.

#### Filosofía
***"ROS, nacido del corazón del código abierto, ofrece libertad y flexibilidad para que los usuarios moldeen su propia realidad robótica, trazando un camino lleno de posibilidades infinitas en el vasto horizonte de la tecnología"***.

#### Diferencias entre ROS1 y ROS2

| Característica        | ROS1          | ROS2        |
|-----------------------|---------------|-------------|
| **Arquitectura**  | Basada en un sistema de nodos con comunicación XML-RPC y TCP/IP | Arquitectura modular y distribuida, comunicación basada en DDS    |
| **Lenguajes de Programación** | Soporte para C++, Python, Lisp, entre otros                   | Soporte para varios lenguajes, incluyendo C++, Python, y más      |
| **Rendimiento** | Limitaciones en rendimiento, seguridad y escalabilidad         | Mejoras significativas en rendimiento, seguridad y escalabilidad  |
| **Multiplataforma** | Principalmente enfocado en Linux                               | Soporte multiplataforma incluyendo Linux, Windows, y macOS        |
| **Herramientas**  | Herramientas de desarrollo y depuración limitadas              | Mejoras en herramientas de depuración, simulación, y gestión de paquetes |
| **Compatibilidad**  | No es directamente compatible con ROS 2                        | Introduce puentes y herramientas de migración para la compatibilidad con ROS 1 |
| **Ecosistema**  | Ecosistema consolidado con una amplia comunidad                 | Ecosistema en constante crecimiento con una creciente comunidad de desarrolladores |

## Arquitectura ROS2

La arquitectura de ROS2 se ha diseñado para abordar las limitaciones de ROS1 y proporcionar una plataforma más flexible, escalable y robusta para el desarrollo de aplicaciones robóticas. A continuación, se proporciona una explicación paso a paso de la arquitectura de ROS2:

| Paso  | Descripción  |
|-------|----------------|
| 1. Arquitectura Modular y Distribuida | ROS 2 se basa en una arquitectura modular y distribuida, donde los nodos son componentes independientes que pueden ejecutarse de manera separada.          |
| 2. Comunicación Basada en DDS | Utiliza DDS para la comunicación entre nodos, ofreciendo un rendimiento superior, mayor seguridad y mejor escalabilidad que el sistema de ROS 1.            |
| 3. Nodos                     | Cada nodo en ROS 2 es un proceso independiente que realiza una tarea específica y se comunica con otros nodos intercambiando mensajes a través de DDS.     |
| 4. Middleware (DDS)          | DDS actúa como el middleware que facilita la comunicación entre nodos, proporcionando mecanismos eficientes para la publicación y suscripción de mensajes. |
| 5. Interfaces de Mensajería (IDL) | Utiliza interfaces de definición de lenguaje (IDL) para describir la estructura de los mensajes que se intercambian entre nodos.                        |
| 6. Gestión de Recursos       | Incluye una capa de gestión de recursos para asignar y administrar eficientemente los recursos del sistema, como memoria y procesamiento.                   |
| 7. Soporte Multiplataforma   | Diseñado para ser ejecutado en una variedad de sistemas operativos, incluyendo Linux, Windows y macOS, lo que proporciona mayor flexibilidad y portabilidad.  |

En resumen, la arquitectura de ROS2 se caracteriza por su modularidad, su sistema de comunicación basado en DDS, su soporte multiplataforma y su capacidad para gestionar eficientemente los recursos del sistema. Estas características hacen de ROS2 una plataforma poderosa y versátil para el desarrollo de aplicaciones robóticas modernas.

## Conceptos Fundamentales de ROS2

### NODOS

Los nodos son bloques de código (clases) que se encargan de partes específicas de las actividades del robot. Estos se van a enlazar mediante tópicos, servicios o acciones. Básicamente nos ayudan a crear un sistema modular que se pueda modificar fácilmente y comunicar.

#### Comandos básicos para nodos:
```bash
# Ejecutar un nodo
ros2 run <paquete> <nodo>

# Visualizar nodos en ejecución
ros2 node list

# Información de un nodo
ros2 node info <nombre_nodo>

# Cambiar nombre del nodo
ros2 run <paquete> <nodo> --ros-args --remap __node:=<nuevo_nombre>
```

#### Ejemplo práctico con turtlesim:

1. **Ejecutar un nodo:**
```bash
ros2 run turtlesim turtlesim_node
```
Este comando lanza el nodo que mediante rqt lanza una interfaz gráfica con una tortuga en unas coordenadas específicas.

2. **Ejecutar un segundo nodo:**
```bash
ros2 run turtlesim turtle_teleop_key
```

3. **Visualizar nodos en ejecución:**
```bash
ros2 node list
```
Resultado:
```
/teleop_turtle
/turtlesim
```

4. **Información detallada de un nodo:**
```bash
ros2 node info /turtlesim
```
Resultado:
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

### TÓPICOS

Son canales en los cuales unos nodos publican información y otros se suscriben para recibirla. La relación para la comunicación puede ser de *uno a muchos*(one to many), *muchos a uno*(many to one) y *muchos a muchos*(many to many).

#### Características de los tópicos

- **Definición de Tópicos**: Canales de comunicación identificados por un nombre único.
- **Tipos de Mensajes**: Los mensajes transmitidos a través de los tópicos pueden ser de tipos estándar (std_msgs) o personalizados.
- **Publicación y Suscripción**: Los nodos pueden publicar o suscribirse a un tópico para enviar o recibir mensajes.
- **Comunicación Desacoplada**: La comunicación se realiza de forma asíncrona y desacoplada entre nodos.
- **Calidad de Servicio (QoS)**: Configuraciones de QoS permiten ajustar la durabilidad, fiabilidad, latencia, entre otros aspectos de la comunicación.
- **Jerarquía de Nombres de los Tópicos**: Los nombres de los tópicos pueden ser jerárquicos para organizar la información.
- **Tópicos Privados**: Los nodos pueden usar tópicos privados para encapsular la comunicación dentro de un nodo o grupo de nodos.

#### Comandos básicos para tópicos:
```bash
# Listar tópicos
ros2 topic list

# Listar tópicos con tipos
ros2 topic list -t

# Ver información de un tópico
ros2 topic info <nombre_topico>

# Escuchar mensajes de un tópico
ros2 topic echo <nombre_topico>

# Publicar en un tópico
ros2 topic pub <nombre_topico> <tipo_mensaje> '<datos>'

# Ver frecuencia de publicación
ros2 topic hz <nombre_topico>

# Ver estructura del mensaje
ros2 interface show <tipo_mensaje>
```

#### Tipos de Mensajes Estándar

##### **std_msgs**: Mensajes estándar básicos
- **std_msgs/String**: Un mensaje de texto simple
- **std_msgs/Int32**: Un entero de 32 bits
- **std_msgs/Float32**: Un número de punto flotante de 32 bits

##### **geometry_msgs**: Mensajes de geometría y movimiento

**geometry_msgs/Twist** - *Crucial para Axioma*
```yaml
# Estructura del mensaje de velocidad
Vector3 linear
    float64 x    # Velocidad lineal hacia adelante/atrás (m/s)
    float64 y    # Velocidad lineal lateral (m/s)
    float64 z    # Velocidad lineal vertical (m/s)
Vector3 angular
    float64 x    # Velocidad angular en X (rad/s)
    float64 y    # Velocidad angular en Y (rad/s)
    float64 z    # Velocidad angular en Z (rad/s)
```

**Ejemplo de uso en Axioma:**
```bash
# Mover el robot hacia adelante a 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar el robot a 0.3 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

**geometry_msgs/Pose** - *Posición y orientación*
```yaml
Point position
    float64 x    # Posición X (metros)
    float64 y    # Posición Y (metros)
    float64 z    # Posición Z (metros)
Quaternion orientation
    float64 x    # Componente X del cuaternión
    float64 y    # Componente Y del cuaternión
    float64 z    # Componente Z del cuaternión
    float64 w    # Componente W del cuaternión
```

##### **sensor_msgs**: Mensajes relacionados con sensores

**sensor_msgs/LaserScan** - *Datos del LIDAR*
```yaml
Header header
    builtin_interfaces/Time stamp
    string frame_id          # Frame de referencia del sensor
float32 angle_min            # Ángulo mínimo de escaneo (rad)
float32 angle_max            # Ángulo máximo de escaneo (rad)
float32 angle_increment      # Incremento angular entre mediciones (rad)
float32 time_increment       # Tiempo entre mediciones (segundos)
float32 scan_time           # Tiempo para completar un escaneo (segundos)
float32 range_min           # Distancia mínima válida (metros)
float32 range_max           # Distancia máxima válida (metros)
float32[] ranges            # Array de distancias medidas (metros)
float32[] intensities       # Array de intensidades de retorno
```

**Ejemplo de datos LIDAR del Axioma:**
```bash
# Escuchar datos del LIDAR
ros2 topic echo /scan

# Resultado típico:
header:
  stamp:
    sec: 1634567890
    nanosec: 123456789
  frame_id: "laser_frame"
angle_min: -3.14159265359
angle_max: 3.14159265359
angle_increment: 0.0174532925199
range_min: 0.1
range_max: 10.0
ranges: [2.3, 2.4, 2.5, 2.6, inf, 1.8, ...]
intensities: []
```

##### **nav_msgs**: Mensajes de navegación

**nav_msgs/Odometry** - *Odometría del robot*
```yaml
Header header
    builtin_interfaces/Time stamp
    string frame_id                    # Frame de odometría (típicamente "odom")
string child_frame_id                  # Frame del robot (típicamente "base_link")
geometry_msgs/PoseWithCovariance pose
    Pose pose
        Point position                 # Posición estimada
        Quaternion orientation         # Orientación estimada
    float64[36] covariance            # Matriz de covarianza 6x6
geometry_msgs/TwistWithCovariance twist
    Twist twist
        Vector3 linear                # Velocidad lineal
        Vector3 angular               # Velocidad angular
    float64[36] covariance           # Matriz de covarianza 6x6
```

**Ejemplo de datos de odometría del Axioma:**
```bash
# Escuchar odometría
ros2 topic echo /odom

# Resultado típico:
header:
  stamp:
    sec: 1634567890
    nanosec: 123456789
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position:
      x: 1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.1736
      w: 0.9848
  covariance: [0.1, 0.0, 0.0, ...]
twist:
  twist:
    linear:
      x: 0.2
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1
  covariance: [0.05, 0.0, 0.0, ...]
```

**nav_msgs/OccupancyGrid** - *Mapa de ocupación*
```yaml
Header header
MapMetaData info
    builtin_interfaces/Time map_load_time
    float32 resolution              # Resolución del mapa (m/pixel)
    uint32 width                   # Ancho del mapa (pixels)
    uint32 height                  # Alto del mapa (pixels)
    Pose origin                    # Origen del mapa en el mundo
int8[] data                        # Datos del mapa (-1: desconocido, 0: libre, 100: ocupado)
```

### SERVICIOS

Los servicios son un mecanismo de comunicación que permite a los nodos intercambiar datos de forma *síncrona*. Un nodo (el servidor) puede ofrecer una funcionalidad específica que otros nodos (los clientes) pueden solicitar.

#### Comandos básicos para servicios:
```bash
# Listar servicios
ros2 service list

# Listar servicios con tipos
ros2 service list -t

# Ver tipo de un servicio
ros2 service type <nombre_servicio>

# Llamar a un servicio
ros2 service call <nombre_servicio> <tipo_servicio> '<datos>'

# Ver estructura del servicio
ros2 interface show <tipo_servicio>
```

#### Clasificación de Servicios

**Servicios estándar (std_srvs):**
- **Empty**: Sin datos de solicitud ni respuesta
- **SetBool**: Toma un booleano y devuelve éxito/fallo
- **Trigger**: Sin solicitud, devuelve éxito/fallo con mensaje

**Servicios del sistema (rcl_interfaces):**
- **SetParameters**: Configurar parámetros de un nodo
- **GetParameters**: Obtener parámetros de un nodo
- **ListParameters**: Listar parámetros disponibles

### ACCIONES

Las acciones en ROS 2 permiten a los nodos ejecutar tareas complejas de forma asíncrona, con retroalimentación y capacidad de cancelación. Son útiles para operaciones que requieren tiempo y seguimiento.

#### Componentes de una acción:
1. **Goal**: El objetivo que el cliente envía al servidor
2. **Result**: El resultado final que el servidor devuelve al cliente
3. **Feedback**: Información intermedia durante la ejecución

#### Comandos básicos para acciones:
```bash
# Listar acciones
ros2 action list

# Listar acciones con tipos
ros2 action list -t

# Ver información de una acción
ros2 action info <nombre_accion>

# Enviar un objetivo
ros2 action send_goal <nombre_accion> <tipo_accion> '<datos>'

# Enviar objetivo con feedback
ros2 action send_goal <nombre_accion> <tipo_accion> '<datos>' --feedback
```

### INTERFACES

En ROS 2, las interfaces definen cómo se comunican los nodos entre sí mediante mensajes, servicios o acciones.

#### Comandos para interfaces:
```bash
# Listar todas las interfaces
ros2 interface list

# Mostrar estructura de una interfaz
ros2 interface show <nombre_interfaz>

# Listar solo interfaces de un tipo
ros2 interface list | grep msg
ros2 interface list | grep srv
ros2 interface list | grep action
```

### LAUNCH FILES

Los launch files son scripts en Python que permiten iniciar y configurar múltiples nodos simultáneamente.

#### Ejemplo de launch file para Axioma:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Argumentos del launch
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Nodo del robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Nodo del LIDAR
        Node(
            package='axioma_drivers',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'frame_id': 'laser_frame',
                'scan_topic': '/scan',
            }],
        ),

        # Nodo de control
        Node(
            package='axioma_control',
            executable='control_node',
            name='axioma_control',
            output='screen',
            parameters=[{
                'wheel_separation': 0.3,
                'wheel_radius': 0.05,
            }],
        ),

        # Nodo de navegación
        Node(
            package='axioma_navigation',
            executable='navigation_node',
            name='axioma_navigation',
            output='screen',
        ),
    ])
```

### Librerías

Son las herramientas que permiten la interación entre ROS y el codigo fuente del proyecto construido en determinado lenguaje. Las librerias principales son RCLCPP para C++ y RCLPY para Python, pero hay librerias clientes para todos los gustos y necesidades.

#### [rclcpp](https://docs.ros.org/en/humble/p/rclcpp/index.html)
Es la biblioteca cliente de ROS que proporciona la API canónica de ***C++*** para interactuar con ROS2.

#### [rclpy](https://docs.ros.org/en/humble/p/rclpy/index.html)
Es la biblioteca cliente de ROS que proporciona la API de ***Python*** para interactuar con ROS2.

## 4 Modelo matemático

<div align="center">
  <img src="https://github.com/mrdavidavl/Axioma_robot/blob/main/image/diff.jpeg" alt="Differential Drive Model" width="400">
</div>

### Modelo Cinemático del Robot Diferencial

El robot Axioma.io utiliza un sistema de locomoción diferencial que se basa en dos ruedas motrices independientes. Este modelo matemático describe el comportamiento cinemático del robot.

#### Parámetros del Sistema

- **L**: Distancia entre las ruedas (wheelbase) = 0.3 m
- **R**: Radio de las ruedas = 0.05 m
- **vL**: Velocidad lineal de la rueda izquierda
- **vR**: Velocidad lineal de la rueda derecha
- **v**: Velocidad lineal del robot
- **ω**: Velocidad angular del robot
- **(x, y)**: Posición del robot en el plano
- **θ**: Orientación del robot

#### Odometría

##### Modelo matemático robot Axioma

**Posición del Robot en coordenadas cartesianas:**

    Vx = V·cos(θ)
    Vy = V·sin(θ)

**Ecuaciones de velocidades lineales y angulares:**

    vL = v - (ω·L)/2    # Velocidad rueda izquierda
    vR = v + (ω·L)/2    # Velocidad rueda derecha

    ω = (vR - vL)/L     # Velocidad Angular
    v = (vR + vL)/2     # Velocidad lineal promedio

**Relaciones angulares:**

    ω = 90°/Δt   →   ω = (2π/ticks)/Δt  (rad/s)

**Período y Frecuencia:**

    T = Δt              # Periodo
    f = 1/T             # Frecuencia
    ω = 2π/T            # Velocidad angular
    V = ω·R             # Velocidad lineal

##### Odometría con encoders

**Distancias calculadas por encoders:**
- Dc: distancia central (Posición promedio)
- Dr: distancia rueda derecha
- Dl: distancia rueda izquierda

        Dc = (Dr + Dl)/2

**Actualización de la posición:**

    x' = x + Dc·cos(θ)
    y' = y + Dc·sin(θ)
    θ' = θ + (Dr - Dl)/L

**Cálculo de distancia por encoder:**

    ΔTicks = TickActual - TickAnterior
    D = 2πR·(ΔTicks/N)

donde **N** es el número de ticks por revolución de la rueda (1000 PPR para Axioma).

##### Sistema de Control con Retroalimentación

**Flujo de datos del sistema:**

    Ticks → Distancia → Posición (x,y)
    Ticks → Velocidad → Control de motores

**Control de tiempo de muestreo:**

| Muestra anterior | **Δmuestreo = 10ms** | Muestra actual |
|------------------|----------------------|----------------|
|                  | ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑  |                |

    Δmuestreo = muestreoActual - muestreoAnterior

    if(Δmuestreo > 10ms) ===> Ejecutar acción de Control

#### Matriz de Transformación Homogénea

Para el robot diferencial, la matriz de transformación que relaciona el sistema de coordenadas local del robot con el sistema global es:

```
T = [cos(θ)  -sin(θ)   x]
    [sin(θ)   cos(θ)   y]
    [   0        0     1]
```

#### Jacobiano del Robot

El jacobiano que relaciona las velocidades de las ruedas con las velocidades del robot es:

```
[v]   [R/2   R/2 ] [ωR]
[ω] = [R/L  -R/L ] [ωL]
```

donde ωR y ωL son las velocidades angulares de las ruedas derecha e izquierda respectivamente.

#### Modelo de Control PID

Para el control de velocidad de los motores, se implementa un controlador PID:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Donde:
- **Kp**: Ganancia proporcional = 2.0
- **Ki**: Ganancia integral = 0.5
- **Kd**: Ganancia derivativa = 0.1
- **e(t)**: Error de velocidad = velocidad_deseada - velocidad_actual

## Instalación y Configuración

### Prerrequisitos

Asegúrate de tener instalado:
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS2 Humble Hawksbill**
- **Python 3.10+**
- **Git**

### Instalación de ROS2 Humble

```bash
# Configurar locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Configurar fuentes
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Humble Desktop
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### Instalación de dependencias específicas

#### Opción 1: Instalación manual de todas las dependencias (recomendado antes de clonar)

```bash
# Herramientas de desarrollo ROS2
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete

# Gazebo Classic (versión 11) para simulación
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs

# Robot State Publisher y Joint State Publisher (con GUI para visualización)
sudo apt install -y ros-humble-robot-state-publisher \
                     ros-humble-joint-state-publisher \
                     ros-humble-joint-state-publisher-gui

# Navegación autónoma (Nav2)
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox (Mapeo y Localización Simultánea)
sudo apt install -y ros-humble-slam-toolbox

# Control con Joystick Xbox
sudo apt install -y ros-humble-joy \
                     ros-humble-joy-linux \
                     ros-humble-teleop-twist-joy

# Control con teclado
sudo apt install -y ros-humble-teleop-twist-keyboard

# TF2 (Sistema de Transformaciones)
sudo apt install -y ros-humble-tf2-tools ros-humble-tf-transformations

# Visualización RViz2
sudo apt install -y ros-humble-rviz2

# URDF y Control
sudo apt install -y ros-humble-xacro \
                     ros-humble-ros2-control \
                     ros-humble-ros2-controllers

# Herramientas adicionales
sudo apt install -y ros-humble-rqt-robot-steering
```

#### Comando único para instalar todo

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
                     gazebo ros-humble-gazebo-ros-pkgs \
                     ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
                     ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
                     ros-humble-joy ros-humble-joy-linux ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
                     ros-humble-tf2-tools ros-humble-tf-transformations \
                     ros-humble-rviz2 \
                     ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers \
                     ros-humble-rqt-robot-steering
```

#### Opción 2: Usar rosdep (después de clonar el repositorio)

```bash
# Desde el directorio del workspace
cd ~/ros2/axioma_humble_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **Nota:** La opción 1 asegura que todos los paquetes necesarios estén instalados antes de compilar. La opción 2 instalará las dependencias declaradas en los archivos `package.xml` del proyecto.

### Configuración del entorno

```bash
# Agregar a ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Inicializar rosdep (solo la primera vez)
sudo rosdep init
rosdep update
```

## Comandos para Ejecutar el Proyecto

### 1. Clonar y configurar el workspace

```bash
# Crear workspace
mkdir -p ~/ros2/axioma_humble_ws/src
cd ~/ros2/axioma_humble_ws/src

# Clonar el repositorio
git clone https://github.com/mrdavidavl/Axioma_robot.git .

# Instalar dependencias del proyecto
cd ~/ros2/axioma_humble_ws
rosdep install --from-paths src --ignore-src -r -y

# Compilar el workspace
colcon build

# Configurar el entorno
source install/setup.bash
```

💡 **Tip**: Agrega `source ~/ros2/axioma_humble_ws/install/setup.bash` a tu `~/.bashrc` para cargar automáticamente el workspace.

---

## 🚀 Los 3 Launches Esenciales

Este proyecto cuenta con 3 launch files principales que cubren todo el flujo de trabajo del robot: desde la creación del mapa hasta la navegación autónoma.

### 1️⃣ **SLAM (Mapping)** - Crear mapas nuevos

```bash
ros2 launch axioma_bringup runmap.launch.py
```

**¿Qué hace?**
- Lanza Gazebo con el robot Axioma
- Inicia SLAM Toolbox para mapeo en tiempo real
- Abre RViz2 con vista de SLAM
- Habilita control con joystick Xbox

**Archivos que usa:**
- `axioma_description/worlds/empty.world`
- `axioma_description/models/axioma_v2/model.sdf`
- `axioma_description/urdf/axioma.urdf`
- `axioma_navigation/config/slam_params.yaml`
- `axioma_description/rviz/slam-toolbox.yaml.rviz`

**Cómo usar:**
1. Ejecuta el launch
2. Mueve el robot con el joystick Xbox:
   - **Stick izquierdo** (vertical): Movimiento adelante/atrás
   - **Stick derecho** (horizontal): Rotación izquierda/derecha
   - **Velocidad lineal máxima**: 0.5 m/s
   - **Velocidad angular máxima**: 2.0 rad/s
3. Observa en RViz2 cómo se construye el mapa en tiempo real
4. Cuando termines, usa el launch #2 para guardar el mapa

**Visualización en RViz2:**
- Mapa en construcción (gris/blanco = explorado, negro = obstáculos, gris oscuro = desconocido)
- Robot (modelo 3D)
- Laser scan (puntos rojos del LIDAR)
- Frames TF (odom → base_link → laser_frame)

---

### 2️⃣ **Guardar Mapa** - Exportar mapa creado

```bash
# IMPORTANTE: Ejecuta esto MIENTRAS runmap.launch.py está corriendo
ros2 launch axioma_bringup save_map.launch.py
```

**¿Qué hace?**
- Guarda el mapa actual en `axioma_navigation/maps/mapa.yaml` y `mapa.pgm`

**IMPORTANTE:**
- Ejecuta esto **MIENTRAS** `runmap.launch.py` está corriendo
- El mapa se guarda automáticamente en la ubicación configurada
- Verás un mensaje de confirmación cuando se complete

**Alternativa manual:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2/axioma_humble_ws/src/axioma_navigation/maps/mi_mapa
```

**Resultado:**
Crea dos archivos:
- `mapa.yaml`: Metadatos del mapa (resolución, origen, thresholds)
- `mapa.pgm`: Imagen del mapa en escala de grises

---

### 3️⃣ **Navegación Autónoma** - Usar mapa guardado

```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

**¿Qué hace?**
- Lanza Gazebo con el robot Axioma
- Carga el mapa estático (`mapa.yaml`)
- Inicia **AMCL** para localización
- Inicia **Nav2 Stack** completo:
  - Controller Server (seguimiento de trayectorias)
  - Planner Server (planificación global)
  - Behavior Server (comportamientos de recovery)
  - BT Navigator (árbol de comportamiento)
- Abre RViz2 con herramientas de navegación
- Publica transformada estática `map→odom` inicial

**Archivos que usa:**
- `axioma_description/worlds/empty.world`
- `axioma_description/models/axioma_v2/model.sdf`
- `axioma_description/urdf/axioma.urdf`
- `axioma_navigation/config/nav2_params.yaml`
- `axioma_navigation/maps/mapa.yaml`
- `axioma_description/rviz/navigation.yaml.rviz`

**Cómo usar:**
1. **Prerequisito**: Asegúrate de tener un mapa guardado en `axioma_navigation/maps/mapa.yaml`
2. Ejecuta el launch
3. Espera a que todo cargue (Gazebo + Nav2) - toma ~10-15 segundos
4. En RViz2:
   - **Paso 1**: Usa **"2D Pose Estimate"** (botón con flecha verde en la barra superior)
     - Click en el mapa donde está el robot
     - Arrastra para indicar la orientación
     - Esto inicializa AMCL con la pose del robot
   - **Paso 2**: Usa **"Nav2 Goal"** (botón con bandera en la barra superior)
     - Click en el destino deseado
     - Arrastra para indicar orientación final
     - El robot navegará autónomamente evitando obstáculos
5. Observa la navegación autónoma

**Visualizaciones en RViz2:**
- **Mapa estático** (gris/blanco = libre, negro = obstáculos)
- **Robot** (modelo 3D en base_link)
- **Nube de partículas AMCL** (flechitas rojas - distribución de probabilidad de la pose)
- **Pose estimada AMCL** (flecha grande amarilla/naranja)
- **Laser scan** (puntos rojos del LIDAR)
- **Plan global** (línea verde - ruta completa al objetivo)
- **Plan local** (línea azul - trayectoria inmediata)
- **Costmaps**:
  - Global costmap (mapa con inflación de obstáculos)
  - Local costmap (ventana local para evitación dinámica)
- **Footprint del robot** (polígono que representa el tamaño del robot)

**Parámetros clave de navegación:**
- **Velocidad lineal máxima**: 0.26 m/s
- **Velocidad angular máxima**: 1.0 rad/s
- **Radio del robot**: 0.15 m
- **Radio de inflación**: 0.55 m (distancia de seguridad a obstáculos)
- **Tolerancia XY al objetivo**: 0.15 m
- **Tolerancia de orientación**: 0.25 rad

---

## 📊 Flujo de Trabajo Típico

### Workflow completo: De cero a navegación autónoma

```bash
# ========================================
# PASO 1: Crear un mapa nuevo
# ========================================
# Terminal 1: Lanzar SLAM
ros2 launch axioma_bringup runmap.launch.py

# Mueve el robot con el joystick explorando el entorno
# Observa en RViz2 cómo se construye el mapa

# Terminal 2: Guardar el mapa (cuando hayas explorado suficiente)
ros2 launch axioma_bringup save_map.launch.py

# Verás un mensaje: "Map saved successfully"

# Ctrl+C en Terminal 1 para cerrar SLAM

# ========================================
# PASO 2: Navegar con el mapa creado
# ========================================
# Terminal 1: Lanzar navegación autónoma
ros2 launch axioma_bringup navigation_bringup.launch.py

# En RViz2:
# 1. Click en "2D Pose Estimate" → Click en la posición del robot → Arrastra para orientación
# 2. Click en "Nav2 Goal" → Click en el destino → Arrastra para orientación final
# 3. ¡El robot navega solo evitando obstáculos!
```

---

## 🎮 Control del Robot

### Joystick Xbox (SLAM)
Configurado para modo de mapeo con control manual:

| Control | Función | Valor |
|---------|---------|-------|
| **Stick izquierdo** (vertical) | Movimiento adelante/atrás | ±0.5 m/s |
| **Stick derecho** (horizontal) | Rotación izquierda/derecha | ±2.0 rad/s |
| **Botón A** | Velocidad normal | - |
| **Botón B** | Velocidad reducida (50%) | - |

**Configuración del joystick:**
```bash
# Verificar que el joystick esté conectado
ls /dev/input/js0

# Calibrar joystick (opcional)
jstest-gtk

# Ver eventos del joystick
ros2 topic echo /joy
```

### RViz2 (Navegación)
Herramientas interactivas para navegación:

| Herramienta | Botón | Función |
|-------------|-------|---------|
| **2D Pose Estimate** | 🎯 (flecha verde) | Establecer posición inicial del robot (obligatorio antes de navegar) |
| **Nav2 Goal** | 🚩 (bandera) | Enviar objetivo de navegación |
| **Publish Point** | 📍 | Marcar puntos en el mapa |

**Shortcuts de RViz2:**
- **R**: Reset vista
- **1**: Vista desde arriba (Top view) - recomendado para navegación
- **2**: Vista orbital
- **Mouse scroll**: Zoom in/out
- **Click derecho + arrastrar**: Pan (mover vista)
- **Click rueda + arrastrar**: Rotar vista

---

## 📁 Estructura de Paquetes

```
src/
├── axioma_description/         # Descripción del robot
│   ├── models/                # Modelos SDF para Gazebo
│   │   └── axioma_v2/
│   │       └── model.sdf      # Modelo completo con sensores
│   ├── urdf/                  # Descripción URDF
│   │   └── axioma.urdf        # Cinemática y TF del robot
│   ├── worlds/                # Mundos de Gazebo
│   │   └── empty.world        # Mundo vacío para testing
│   └── rviz/                  # Configuraciones de RViz2
│       ├── slam-toolbox.yaml.rviz      # Vista para SLAM
│       └── navigation.yaml.rviz        # Vista para navegación
│
├── axioma_gazebo/             # Soporte para Gazebo (plugins)
│   └── (vacío - reservado para plugins personalizados)
│
├── axioma_navigation/         # Navegación y SLAM
│   ├── config/
│   │   ├── slam_params.yaml   # Configuración SLAM Toolbox
│   │   └── nav2_params.yaml   # Configuración Nav2 completa
│   │       ├── amcl           # Localización
│   │       ├── controller_server  # Control de trayectorias (DWB)
│   │       ├── planner_server     # Planificación global (NavFn)
│   │       ├── behavior_server    # Comportamientos recovery
│   │       ├── bt_navigator       # Árbol de comportamiento
│   │       ├── global_costmap     # Costmap global
│   │       ├── local_costmap      # Costmap local
│   │       └── velocity_smoother  # Suavizado de velocidades
│   └── maps/
│       ├── mapa.yaml          # Metadatos del mapa
│       └── mapa.pgm           # Imagen del mapa
│
└── axioma_bringup/            # Launches principales
    └── launch/
        ├── runmap.launch.py              # 🗺️  SLAM (Mapeo)
        ├── save_map.launch.py            # 💾 Guardar mapa
        └── navigation_bringup.launch.py  # 🚀 Navegación autónoma
```

---

## ⚙️ Configuración Avanzada

### AMCL (Localización)

Configuración optimizada para tracking preciso:

```yaml
# axioma_navigation/config/nav2_params.yaml
amcl:
  ros__parameters:
    max_particles: 5000          # Aumentado para mejor localización
    min_particles: 1000
    update_min_d: 0.1            # Actualiza cada 10cm de movimiento
    update_min_a: 0.1            # Actualiza cada 0.1 rad de rotación
    alpha1-5: 0.05               # Confianza en odometría (menor = más confianza)
    recovery_alpha_fast: 0.1     # Recovery automático si se pierde
    recovery_alpha_slow: 0.001
```

**¿Cómo funciona AMCL?**
- Usa un filtro de partículas (Monte Carlo) para estimar la pose del robot
- Compara el laser scan con el mapa para ajustar las partículas
- Más partículas = mejor precisión pero más CPU
- Las partículas convergen a la pose más probable

### DWB Local Planner (Controller)

Configuración para seguimiento suave de trayectorias:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.26                # Velocidad lineal máxima
  max_vel_theta: 1.0             # Velocidad angular máxima
  sim_time: 1.7                  # Mira 1.7s adelante
  vx_samples: 20                 # Muestras de velocidad lineal
  vtheta_samples: 20             # Muestras de velocidad angular
```

**Critics (evaluadores de trayectorias):**
1. **RotateToGoal**: Prioriza rotar hacia el objetivo cuando está cerca
2. **PathAlign**: Mantenerse alineado con el plan global
3. **PathDist**: Minimizar distancia al plan global
4. **GoalAlign**: Alinearse con la orientación del objetivo
5. **GoalDist**: Minimizar distancia al objetivo
6. **Oscillation**: Evitar oscilaciones
7. **BaseObstacle**: Evitar colisiones

### Costmaps

**Global Costmap** (mapa completo):
```yaml
global_costmap:
  resolution: 0.05               # 5cm por píxel
  robot_radius: 0.15             # Radio del robot
  inflation_radius: 0.55         # Distancia de seguridad
  plugins: [static_layer, obstacle_layer, inflation_layer]
```

**Local Costmap** (ventana local):
```yaml
local_costmap:
  rolling_window: true           # Ventana móvil centrada en el robot
  width: 3                       # 3 metros de ancho
  height: 3                      # 3 metros de alto
  resolution: 0.05
  plugins: [voxel_layer, inflation_layer]
```

**Capas del costmap:**
- **static_layer**: Mapa estático cargado desde mapa.yaml
- **obstacle_layer**: Obstáculos detectados por sensores
- **voxel_layer**: Obstáculos 3D (versión mejorada de obstacle_layer)
- **inflation_layer**: Infla obstáculos para crear zona de seguridad

---

## 🐛 Solución de Problemas

### El mapa no se ve en RViz2 (navegación)

**Síntomas:**
- El robot aparece en RViz2 pero no el mapa
- Warning: "Map received but not displayed"

**Causas y soluciones:**
1. **Fixed Frame incorrecto**
   - En RViz2 → Global Options → Fixed Frame → Cambiar a `map`

2. **QoS mismatch del topic /map**
   - En RViz2 → Add → By topic → /map → Map
   - Click en Map → Durability Policy → `Transient Local`

3. **El mapa no existe**
   ```bash
   # Verificar que el mapa exista
   ls ~/ros2/axioma_humble_ws/src/axioma_navigation/maps/
   # Debe mostrar: mapa.yaml y mapa.pgm
   ```

4. **Map server no está publicando**
   ```bash
   # Verificar que el map server esté corriendo
   ros2 node list | grep map_server

   # Verificar que publica el mapa
   ros2 topic echo /map --once
   ```

---

### El robot se pierde en navegación

**Síntomas:**
- Las partículas de AMCL se dispersan mucho
- El robot gira sin control
- Warning: "AMCL cannot transform from odom to map"

**Causas y soluciones:**
1. **No se estableció pose inicial**
   - En RViz2 → Click en "2D Pose Estimate"
   - Click en la posición del robot y arrastra para orientación

2. **Pocas partículas de AMCL**
   ```yaml
   # Editar: axioma_navigation/config/nav2_params.yaml
   amcl:
     max_particles: 5000  # Aumentar si el robot se pierde
     min_particles: 1000
   ```

3. **Odometría de mala calidad**
   ```yaml
   # Reducir confianza en odometría (valores más altos)
   amcl:
     alpha1: 0.2  # Era 0.05
     alpha2: 0.2
     alpha3: 0.2
     alpha4: 0.2
   ```

4. **Recovery behaviors**
   ```bash
   # Ver estado de AMCL
   ros2 topic echo /amcl/particle_cloud

   # Forzar reinicialización global
   ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
   ```

---

### El robot sale del área mapeada

**Síntomas:**
- El robot planifica rutas fuera del mapa
- Entra en zonas desconocidas (gris oscuro) y se pierde

**Causas y soluciones:**
1. **Planner permite zonas desconocidas**
   ```yaml
   # Editar: axioma_navigation/config/nav2_params.yaml
   planner_server:
     GridBased:
       allow_unknown: false  # No permitir rutas por desconocido
   ```

2. **Costmap trata desconocido como libre**
   ```yaml
   # Cambiar unknown_cost_value en global_costmap
   global_costmap:
     unknown_cost_value: 255  # Tratar desconocido como obstáculo
     # Era: unknown_cost_value: 0 (tratar como libre)
   ```

3. **Objetivos fuera del mapa**
   - En RViz2, asegúrate de colocar el "Nav2 Goal" dentro del área blanca/gris clara del mapa
   - Evita colocar objetivos en el borde del mapa

---

### SLAM no actualiza el mapa

**Síntomas:**
- El mapa no cambia aunque el robot se mueva
- LIDAR muestra datos pero no se agrega al mapa

**Causas y soluciones:**
1. **El robot no se está moviendo**
   - SLAM necesita movimiento para triangular
   - Mueve el robot con el joystick

2. **LIDAR no está publicando datos**
   ```bash
   # Verificar topic del LIDAR
   ros2 topic hz /scan
   # Debe mostrar ~10 Hz

   # Ver datos del LIDAR
   ros2 topic echo /scan
   ```

3. **Frame_id del LIDAR incorrecto**
   ```bash
   # Verificar transformadas
   ros2 run tf2_ros tf2_echo base_link laser_frame
   # Debe mostrar la transformada
   ```

4. **Parámetros de SLAM muy conservadores**
   ```yaml
   # Editar: axioma_navigation/config/slam_params.yaml
   slam_toolbox:
     minimum_travel_distance: 0.2  # Reducir para actualizar más frecuente
     minimum_travel_heading: 0.2
   ```

---

### Nav2 dice "No valid path"

**Síntomas:**
- Al enviar un objetivo, aparece error "Failed to find a valid path"
- El robot no se mueve

**Causas y soluciones:**
1. **Objetivo fuera del mapa o bloqueado**
   - Verifica que el objetivo esté en área blanca (libre)
   - No debe haber obstáculos bloqueando completamente el camino

2. **Inflación de costmap demasiado grande**
   ```yaml
   # Reducir inflación temporalmente
   global_costmap:
     inflation_radius: 0.3  # Era 0.55
   ```

3. **Tolerancia del planner muy estricta**
   ```yaml
   planner_server:
     GridBased:
       tolerance: 0.5  # Aumentar tolerancia
   ```

4. **Pose inicial muy mala**
   - Re-establecer pose con "2D Pose Estimate"
   - Asegúrate de que las partículas AMCL estén convergidas

---

### Joystick Xbox no responde

**Síntomas:**
- El robot no se mueve al mover los sticks
- `/joy` topic no publica datos

**Causas y soluciones:**
1. **Joystick no detectado**
   ```bash
   # Verificar dispositivo
   ls /dev/input/js*
   # Debe mostrar: /dev/input/js0

   # Dar permisos
   sudo chmod a+rw /dev/input/js0
   ```

2. **Nodo joy no está corriendo**
   ```bash
   # Verificar nodo
   ros2 node list | grep joy

   # Reiniciar nodo manualmente
   ros2 run joy joy_node
   ```

3. **Mapeo de botones incorrecto**
   ```bash
   # Ver datos raw del joystick
   ros2 topic echo /joy

   # Nota los índices de los axes que usas
   # Editar el launch si es necesario
   ```

---

### Gazebo no inicia o crashea

**Síntomas:**
- Ventana de Gazebo no aparece
- Error: "Gazebo died with error code -11"

**Causas y soluciones:**
1. **GPU no soportada o drivers viejos**
   ```bash
   # Verificar OpenGL
   glxinfo | grep OpenGL

   # Lanzar Gazebo sin GPU acceleration
   export LIBGL_ALWAYS_SOFTWARE=1
   ros2 launch axioma_bringup navigation_bringup.launch.py
   ```

2. **Instancia previa no cerrada**
   ```bash
   # Matar procesos de Gazebo
   killall gzserver
   killall gzclient
   ```

3. **Modelo SDF corrupto**
   ```bash
   # Verificar modelo
   gz sdf -k ~/ros2/axioma_humble_ws/src/axioma_description/models/axioma_v2/model.sdf
   ```

---

### Errores de transformadas (TF)

**Síntomas:**
- Warning: "Transform from base_link to map failed"
- Error: "Could not transform from odom to map"

**Causas y soluciones:**
1. **Transformada faltante**
   ```bash
   # Ver árbol de transformadas
   ros2 run tf2_tools view_frames
   evince frames.pdf

   # Debe mostrar: map → odom → base_link → laser_frame
   ```

2. **use_sim_time desincronizado**
   ```bash
   # Todos los nodos deben tener use_sim_time: True en simulación
   ros2 param list /nombre_nodo
   ros2 param get /nombre_nodo use_sim_time
   # Debe devolver: Boolean value is: True
   ```

3. **Retraso en transformadas**
   ```yaml
   # Aumentar tolerancia en nav2_params.yaml
   controller_server:
     transform_tolerance: 0.5  # Era 0.2
   ```

---

### Errores de paquetes faltantes

**Síntomas:**
- Error al compilar: "Could not find a package configuration file"
- Error al lanzar: "Package 'nombre_paquete' not found"
- Error específico: `package 'joint_state_publisher_gui' not found`

**Causas y soluciones:**
1. **Paquetes ROS2 no instalados**
   ```bash
   # Instalar todas las dependencias automáticamente
   cd ~/ros2/axioma_humble_ws
   rosdep install --from-paths src --ignore-src -r -y

   # O instalar manualmente el paquete faltante (ejemplo con joint_state_publisher_gui)
   sudo apt install ros-humble-joint-state-publisher-gui
   ```

2. **Verificar que ROS2 Humble esté instalado correctamente**
   ```bash
   # Verificar instalación de ROS2
   ros2 --version
   # Debe mostrar: ros2 cli version X.X.X

   # Verificar que el setup esté sourced
   source /opt/ros/humble/setup.bash
   ```

3. **Paquete específico no instalado**
   ```bash
   # Buscar paquete disponible
   apt search ros-humble-nombre-paquete

   # Instalar paquete específico
   sudo apt install ros-humble-nombre-paquete
   ```

4. **Compilar después de instalar dependencias**
   ```bash
   cd ~/ros2/axioma_humble_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

> **Tip:** Si acabas de clonar el repositorio, asegúrate de instalar TODAS las dependencias usando el comando único de instalación de la sección de dependencias antes de compilar.

---

## 🔧 Monitoreo y Depuración

### Comandos útiles para debugging

```bash
# ========================================
# VERIFICACIÓN DE NODOS
# ========================================
# Listar todos los nodos activos
ros2 node list

# Info detallada de un nodo (topics, services, actions)
ros2 node info /amcl
ros2 node info /controller_server

# ========================================
# VERIFICACIÓN DE TOPICS
# ========================================
# Listar todos los topics
ros2 topic list

# Ver frecuencia de publicación
ros2 topic hz /scan        # LIDAR (~10 Hz)
ros2 topic hz /odom        # Odometría (~50 Hz)
ros2 topic hz /cmd_vel     # Comandos de velocidad

# Ver datos de un topic
ros2 topic echo /scan --once
ros2 topic echo /odom
ros2 topic echo /amcl_pose

# Información del topic (publishers, subscribers, QoS)
ros2 topic info /map
ros2 topic info /scan

# ========================================
# VERIFICACIÓN DE TRANSFORMADAS
# ========================================
# Ver transformada entre dos frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame

# Generar gráfico del árbol TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitorear transformadas en tiempo real
ros2 run tf2_ros tf2_monitor

# ========================================
# VERIFICACIÓN DE PARÁMETROS
# ========================================
# Listar parámetros de un nodo
ros2 param list /amcl

# Obtener valor de un parámetro
ros2 param get /amcl max_particles
ros2 param get /controller_server use_sim_time

# Cambiar parámetro en tiempo real
ros2 param set /controller_server max_vel_x 0.3

# ========================================
# VERIFICACIÓN DE SERVICIOS
# ========================================
# Listar servicios
ros2 service list | grep amcl

# Llamar a un servicio (ejemplo: limpiar costmap)
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# Reinicializar AMCL
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# ========================================
# VERIFICACIÓN DE ACCIONES
# ========================================
# Listar acciones
ros2 action list

# Info de una acción
ros2 action info /navigate_to_pose

# Enviar objetivo de navegación por CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# ========================================
# LOGS Y DIAGNÓSTICO
# ========================================
# Ver logs de un nodo específico
ros2 run rqt_console rqt_console

# Nivel de logging
ros2 run rqt_logger_level rqt_logger_level

# Ver gráfico de nodos y topics
rqt_graph

# ========================================
# PERFORMANCE
# ========================================
# Monitorear uso de CPU/RAM de nodos
htop
# Buscar procesos: ros, gazebo, rviz

# Ver bandwidth de topics
ros2 topic bw /scan
ros2 topic bw /map

# ========================================
# DIAGNÓSTICO ESPECÍFICO
# ========================================
# Verificar que SLAM esté activo
ros2 topic echo /map_metadata --once

# Verificar nube de partículas AMCL
ros2 topic echo /particle_cloud --once

# Ver plan global calculado
ros2 topic echo /plan

# Ver costmap global
ros2 topic echo /global_costmap/costmap --once

# Estadísticas de Nav2
ros2 topic echo /bt_navigator/transition_event
```

### Herramientas gráficas de diagnóstico

```bash
# Consola de ROS (logs coloridos y filtrados)
ros2 run rqt_console rqt_console

# Gráfico de nodos y topics en tiempo real
rqt_graph

# Publicar mensajes manualmente (útil para testing)
ros2 run rqt_publisher rqt_publisher

# Monitor de topics (ver valores en tiempo real)
ros2 run rqt_topic rqt_topic

# Calibrar parámetros dinámicamente
ros2 run rqt_reconfigure rqt_reconfigure

# Plot de datos en tiempo real
ros2 run rqt_plot rqt_plot /odom/twist/twist/linear/x

# Visualizar transformadas TF
ros2 run rqt_tf_tree rqt_tf_tree

# Ver imágenes de cámaras (si usas cámara)
ros2 run rqt_image_view rqt_image_view
```

---

## 📊 Parámetros Clave del Sistema

### Cinemática del Robot

```yaml
# Parámetros físicos
wheel_separation: 0.3 m          # Distancia entre ruedas
wheel_radius: 0.05 m             # Radio de las ruedas
robot_radius: 0.15 m             # Radio del robot (para costmaps)

# Límites de velocidad
max_linear_velocity: 0.5 m/s     # SLAM mode
max_linear_velocity: 0.26 m/s    # Navigation mode (más conservador)
max_angular_velocity: 2.0 rad/s  # SLAM mode
max_angular_velocity: 1.0 rad/s  # Navigation mode

# Aceleraciones
max_linear_acceleration: 2.5 m/s²
max_angular_acceleration: 3.2 rad/s²
```

### SLAM Toolbox

```yaml
# Modo de operación
mode: mapping                    # mapping / localization

# Frecuencia de actualización
map_update_interval: 2.0         # Segundos entre actualizaciones del mapa

# Umbrales de movimiento para actualizar
minimum_travel_distance: 0.5     # Metros
minimum_travel_heading: 0.5      # Radianes

# Resolución del mapa
resolution: 0.05                 # Metros por píxel (5cm)

# Tamaño del mapa
map_size: 2048                   # Píxeles (2048 * 0.05 = 102.4m)
```

### AMCL (Localización)

```yaml
# Cantidad de partículas
max_particles: 5000              # Máximo número de partículas
min_particles: 1000              # Mínimo número de partículas

# Modelo de movimiento (menor = más confianza en odometría)
alpha1: 0.05                     # Ruido de rotación por rotación
alpha2: 0.05                     # Ruido de rotación por traslación
alpha3: 0.05                     # Ruido de traslación por traslación
alpha4: 0.05                     # Ruido de traslación por rotación

# Actualización
update_min_d: 0.1                # Actualizar cada 10cm
update_min_a: 0.1                # Actualizar cada 0.1 rad

# Laser
max_beams: 120                   # Número de rayos LIDAR a procesar
laser_max_range: 10.0            # Rango máximo del LIDAR
```

### Nav2 Controller (DWB)

```yaml
# Velocidades
max_vel_x: 0.26 m/s
max_vel_theta: 1.0 rad/s
min_vel_x: 0.0                   # Puede detenerse completamente
min_speed_xy: 0.0
min_speed_theta: 0.0

# Aceleraciones
acc_lim_x: 2.5 m/s²
acc_lim_theta: 3.2 rad/s²
decel_lim_x: -2.5 m/s²
decel_lim_theta: -3.2 rad/s²

# Simulación de trayectorias
sim_time: 1.7 s                  # Tiempo de predicción
vx_samples: 20                   # Muestras de velocidad lineal
vtheta_samples: 20               # Muestras de velocidad angular

# Tolerancia al objetivo
xy_goal_tolerance: 0.15 m
yaw_goal_tolerance: 0.25 rad
```

### Costmaps

```yaml
# Resolución
resolution: 0.05 m               # 5cm por píxel

# Inflación de obstáculos
inflation_radius: 0.55 m         # Radio de zona de seguridad
cost_scaling_factor: 3.0         # Factor de decaimiento del costo

# Tamaño del local costmap
width: 3 m
height: 3 m
rolling_window: true             # Centrado en el robot

# Frecuencias
update_frequency: 5.0 Hz         # Actualización del costmap
publish_frequency: 2.0 Hz        # Publicación del costmap
```

---

## 🚀 Desarrollo con ROS2

### Creando paquetes personalizados

#### Paquete Python
```bash
cd ~/ros2/axioma_humble_ws/src

# Crear paquete Python
ros2 pkg create --build-type ament_python --node-name mi_nodo mi_paquete_python

# Estructura creada:
# mi_paquete_python/
# ├── mi_paquete_python/
# │   ├── __init__.py
# │   └── mi_nodo.py
# ├── package.xml
# ├── setup.py
# ├── setup.cfg
# └── resource/

# Compilar paquete específico
cd ~/ros2/axioma_humble_ws
colcon build --packages-select mi_paquete_python
source install/setup.bash

# Ejecutar el nodo
ros2 run mi_paquete_python mi_nodo
```

#### Paquete C++
```bash
cd ~/ros2/axioma_humble_ws/src

# Crear paquete C++
ros2 pkg create --build-type ament_cmake --node-name mi_nodo_cpp mi_paquete_cpp

# Estructura creada:
# mi_paquete_cpp/
# ├── src/
# │   └── mi_nodo_cpp.cpp
# ├── include/mi_paquete_cpp/
# ├── CMakeLists.txt
# └── package.xml

# Compilar paquete específico
cd ~/ros2/axioma_humble_ws
colcon build --packages-select mi_paquete_cpp
source install/setup.bash

# Ejecutar el nodo
ros2 run mi_paquete_cpp mi_nodo_cpp
```

### APIs principales

#### Python (rclpy)

| Método | Descripción | Ejemplo |
|--------|-------------|---------|
| `rclpy.init()` | Inicializa rclpy | `rclpy.init()` |
| `Node()` | Crea un nodo | `node = Node('axioma_node')` |
| `create_subscription()` | Crea suscriptor | `sub = node.create_subscription(LaserScan, '/scan', callback, 10)` |
| `create_publisher()` | Crea publicador | `pub = node.create_publisher(Twist, '/cmd_vel', 10)` |
| `create_timer()` | Crea timer | `timer = node.create_timer(0.1, callback)` |
| `spin()` | Mantiene nodo activo | `rclpy.spin(node)` |
| `get_logger()` | Logger del nodo | `self.get_logger().info('Axioma iniciado')` |

#### Ejemplo de nodo en Python para Axioma:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AxiomaAvoidance(Node):
    """
    Nodo de evitación simple de obstáculos para Axioma.
    Si detecta un obstáculo a menos de 0.5m, el robot gira.
    """

    def __init__(self):
        super().__init__('axioma_avoidance')

        # Publisher para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber para datos del LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Timer para control periódico (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables de estado
        self.obstacle_detected = False
        self.min_distance = float('inf')

        self.get_logger().info('Axioma Avoidance Node iniciado')

    def scan_callback(self, msg: LaserScan):
        """Procesar datos del LIDAR"""
        # Filtrar valores inválidos (inf, nan)
        valid_ranges = [r for r in msg.ranges
                        if msg.range_min < r < msg.range_max]

        if valid_ranges:
            self.min_distance = min(valid_ranges)

            # Detectar obstáculo si está a menos de 0.5m
            if self.min_distance < 0.5:
                self.obstacle_detected = True
                self.get_logger().warn(
                    f'⚠️  Obstáculo detectado a {self.min_distance:.2f}m'
                )
            else:
                self.obstacle_detected = False

    def control_loop(self):
        """Lógica de control del robot"""
        cmd = Twist()

        if self.obstacle_detected:
            # Obstáculo detectado: Detenerse y girar
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Girar a la derecha
            self.get_logger().info('🔄 Girando para evitar obstáculo')
        else:
            # Sin obstáculos: Avanzar
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            # self.get_logger().info('➡️  Avanzando')

        # Publicar comando
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AxiomaAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener robot antes de cerrar
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Para usar este nodo:**
```bash
# Guardar como: mi_paquete_python/mi_paquete_python/avoidance_node.py

# Agregar a setup.py:
entry_points={
    'console_scripts': [
        'avoidance = mi_paquete_python.avoidance_node:main',
    ],
},

# Compilar y ejecutar
colcon build --packages-select mi_paquete_python
source install/setup.bash
ros2 run mi_paquete_python avoidance
```

#### C++ (rclcpp)

| Método | Descripción | Ejemplo |
|--------|-------------|---------|
| `rclcpp::init()` | Inicializa rclcpp | `rclcpp::init(argc, argv)` |
| `std::make_shared<Node>()` | Crea nodo | `auto node = std::make_shared<rclcpp::Node>("axioma")` |
| `create_subscription()` | Crea suscriptor | `auto sub = create_subscription<LaserScan>("/scan", 10, callback)` |
| `create_publisher()` | Crea publicador | `auto pub = create_publisher<Twist>("/cmd_vel", 10)` |
| `create_wall_timer()` | Crea timer | `timer = create_wall_timer(100ms, callback)` |
| `spin()` | Mantiene nodo activo | `rclcpp::spin(node)` |

#### Ejemplo de nodo en C++ para Axioma:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class AxiomaAvoidance : public rclcpp::Node
{
public:
    AxiomaAvoidance() : Node("axioma_avoidance")
    {
        // Publisher para comandos de velocidad
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber para datos del LIDAR
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AxiomaAvoidance::scan_callback, this, std::placeholders::_1)
        );

        // Timer para control periódico (10 Hz)
        timer_ = create_wall_timer(
            100ms,
            std::bind(&AxiomaAvoidance::control_loop, this)
        );

        RCLCPP_INFO(get_logger(), "Axioma Avoidance Node iniciado");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Filtrar valores válidos
        std::vector<float> valid_ranges;
        for (const auto& range : msg->ranges) {
            if (range > msg->range_min && range < msg->range_max) {
                valid_ranges.push_back(range);
            }
        }

        if (!valid_ranges.empty()) {
            min_distance_ = *std::min_element(valid_ranges.begin(), valid_ranges.end());

            // Detectar obstáculo
            if (min_distance_ < 0.5) {
                obstacle_detected_ = true;
                RCLCPP_WARN(get_logger(), "⚠️  Obstáculo detectado a %.2fm", min_distance_);
            } else {
                obstacle_detected_ = false;
            }
        }
    }

    void control_loop()
    {
        auto cmd = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {
            // Obstáculo: Girar
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
            RCLCPP_INFO(get_logger(), "🔄 Girando para evitar obstáculo");
        } else {
            // Sin obstáculos: Avanzar
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool obstacle_detected_ = false;
    float min_distance_ = std::numeric_limits<float>::infinity();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AxiomaAvoidance>());
    rclcpp::shutdown();
    return 0;
}
```

**CMakeLists.txt para compilar:**
```cmake
cmake_minimum_required(VERSION 3.8)
project(mi_paquete_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(avoidance_node src/avoidance_node.cpp)
ament_target_dependencies(avoidance_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Install
install(TARGETS
  avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

### Launch Files Avanzados

#### Launch file completo para Axioma con múltiples nodos:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # ===========================================
    # ARGUMENTS
    # ===========================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('axioma_description'),
            'worlds',
            'empty.world'
        ]),
        description='Path to Gazebo world file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('axioma_description'),
            'rviz',
            'navigation.yaml.rviz'
        ]),
        description='Path to RViz config file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    # ===========================================
    # GAZEBO
    # ===========================================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # ===========================================
    # ROBOT
    # ===========================================
    # Cargar URDF
    urdf_file = os.path.join(
        FindPackageShare('axioma_description').find('axioma_description'),
        'urdf',
        'axioma.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc
        }]
    )

    # Spawn robot in Gazebo (delayed 3 seconds)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'axioma',
                    '-file', PathJoinSubstitution([
                        FindPackageShare('axioma_description'),
                        'models',
                        'axioma_v2',
                        'model.sdf'
                    ]),
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # ===========================================
    # NAVIGATION
    # ===========================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                FindPackageShare('axioma_navigation'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # Map server + AMCL (localization)
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': PathJoinSubstitution([
                FindPackageShare('axioma_navigation'),
                'maps',
                'mapa.yaml'
            ]),
            'params_file': PathJoinSubstitution([
                FindPackageShare('axioma_navigation'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # ===========================================
    # RVIZ2
    # ===========================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ===========================================
    # CUSTOM NODES (EJEMPLO)
    # ===========================================
    # Nodo personalizado de evitación de obstáculos
    avoidance_node = Node(
        package='mi_paquete_python',
        executable='avoidance',
        name='axioma_avoidance',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ===========================================
    # LAUNCH DESCRIPTION
    # ===========================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_file_arg,
        rviz_config_arg,
        use_rviz_arg,

        # Gazebo
        gazebo_server,
        gazebo_client,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Navigation
        localization_bringup,
        nav2_bringup,

        # Visualization
        rviz,

        # Custom nodes
        # avoidance_node,  # Descomentar para usar
    ])
```

**Para usar este launch:**
```bash
# Guardar como: axioma_bringup/launch/custom_bringup.launch.py

# Ejecutar con argumentos personalizados
ros2 launch axioma_bringup custom_bringup.launch.py

# Ejecutar sin RViz
ros2 launch axioma_bringup custom_bringup.launch.py use_rviz:=false

# Ejecutar con mundo personalizado
ros2 launch axioma_bringup custom_bringup.launch.py world:=/path/to/my_world.world
```

---

## 🚀 Características del Sistema

- ✅ **Navegación autónoma** con planificación de rutas usando Nav2
- ✅ **SLAM** (Simultaneous Localization and Mapping) con slam_toolbox
- ✅ **Evitación de obstáculos** en tiempo real con LIDAR
- ✅ **Control diferencial** con odometría de encoders de alta precisión
- ✅ **Interfaz de visualización** en RViz2 con datos en tiempo real
- ✅ **Simulación completa** en Gazebo con física realista
- ✅ **Comunicación inalámbrica** para monitoreo y control remoto
- ✅ **Arquitectura modular** con nodos especializados
- ✅ **Control PID** para velocidades de motores
- ✅ **Telemetría completa** del estado del robot
- ✅ **QoS configurables** para comunicación robusta
- ✅ **Launch files parametrizados** para fácil configuración
- ✅ **Soporte para ROS2 Humble** en Ubuntu 22.04

---

## 🤝 Contribuir

¡Las contribuciones son bienvenidas! Si deseas mejorar el proyecto:

1. **Fork** el repositorio
2. Crea una **rama** para tu feature:
   ```bash
   git checkout -b feature/MiNuevaCaracteristica
   ```
3. **Commit** tus cambios:
   ```bash
   git commit -m 'Add: Nueva característica increíble'
   ```
4. **Push** a la rama:
   ```bash
   git push origin feature/MiNuevaCaracteristica
   ```
5. Abre un **Pull Request** describiendo tus cambios

### Áreas donde puedes contribuir:
- 🐛 Reportar bugs o issues
- 💡 Sugerir nuevas features
- 📝 Mejorar documentación
- 🧪 Agregar tests unitarios
- 🎨 Mejorar configuraciones de RViz
- 🗺️ Crear nuevos mundos de Gazebo
- 🤖 Implementar nuevos comportamientos de navegación
- 📊 Optimizar parámetros de Nav2

---

## 📝 Licencia

Este proyecto está bajo la **Licencia BSD** - ver el archivo [LICENSE](LICENSE) para más detalles.

La Licencia BSD permite:
- ✅ Uso comercial
- ✅ Modificación
- ✅ Distribución
- ✅ Uso privado

---

## 👥 Autores y Agradecimientos

### Autores
- **Mario David Alvarez Vallejo** - *Desarrollo principal* - [@mrdavidavl](https://github.com/mrdavidavl)
  - 📧 [ing.marioalvarezvallejo@gmail.com](mailto:ing.marioalvarezvallejo@gmail.com)
  - 📱 Instagram: [@mrdavidavl](https://instagram.com/mrdavidavl)
  - 🐦 X (Twitter): [@mrdavidavl](https://x.com/mrdavidavl)
- **Semillero de Robótica SIRO** - *Colaboradores y testers*
  - Universidad de Bogotá Jorge Tadeo Lozano

### Agradecimientos
- 🎓 **Semillero de Robótica SIRO** - Por el apoyo y recursos
- 🏫 **Universidad de Bogotá Jorge Tadeo Lozano** - Por las facilidades
- 🤖 **Comunidad ROS2** - Por el increíble framework
- 🔧 **Open Source Robotics Foundation** - Por mantener ROS2
- 🏆 **Mercury Robotics Challenge** - Por la inspiración competitiva
- 💻 **Comunidad de GitHub** - Por las contribuciones y feedback

---

## 🎯 Enlaces Útiles

### Documentación Oficial
- 📘 **ROS2 Humble Documentation** - [docs.ros.org/en/humble](https://docs.ros.org/en/humble/)
- 🧭 **Nav2 Documentation** - [navigation.ros.org](https://navigation.ros.org/)
- 🗺️ **SLAM Toolbox** - [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- 🎮 **Gazebo Documentation** - [gazebosim.org/docs](http://gazebosim.org/docs)
- 👁️ **RViz2 Documentation** - [github.com/ros2/rviz](https://github.com/ros2/rviz)

### Tutoriales
- 🚀 **ROS2 Tutorials** - [docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- 📖 **Nav2 Tutorials** - [navigation.ros.org/tutorials](https://navigation.ros.org/tutorials/index.html)
- 🎓 **The Construct ROS2 Courses** - [theconstructsim.com](https://www.theconstructsim.com/)

### Comunidad
- 💬 **ROS Discourse** - [discourse.ros.org](https://discourse.ros.org/)
- 🐙 **ROS2 GitHub** - [github.com/ros2](https://github.com/ros2)
- 📺 **ROS Developers YouTube** - Tutoriales en video

### Universidad y Proyectos
- 🏛️ **Universidad Jorge Tadeo Lozano** - [utadeo.edu.co](https://www.utadeo.edu.co/)
- 🤖 **Semillero SIRO** - Proyectos de robótica educativa
- 🏆 **Mercury Robotics Challenge** - Competencia latinoamericana

---

## 📧 Contacto

<div align="center">

¿Tienes preguntas o sugerencias? ¡Contáctanos!

| Canal | Link |
|-------|------|
| 📧 **Email Personal** | [ing.marioalvarezvallejo@gmail.com](mailto:ing.marioalvarezvallejo@gmail.com) |
| 🐙 **GitHub Personal** | [@mrdavidavl](https://github.com/mrdavidavl) |
| 🤖 **SIRO GitHub** | [TadeoRoboticsGroup](https://github.com/TadeoRoboticsGroup) |
| 💬 **Issues** | [Reportar un problema](https://github.com/mrdavidavl/Axioma_robot/issues) |
| 🎓 **Universidad** | [Universidad Jorge Tadeo Lozano](https://www.utadeo.edu.co/) |
| 📱 **Instagram** | [@mrdavidavl](https://instagram.com/mrdavidavl) |
| 🐦 **X (Twitter)** | [@mrdavidavl](https://x.com/mrdavidavl) |
| 📘 **Facebook** | [@mrdavidavl](https://facebook.com/mrdavidavl) |

</div>

---

## 🌟 Apóyanos

Si este proyecto te resultó útil, considera:
- ⭐ Darle una **estrella** en GitHub
- 🔄 **Compartir** con otros estudiantes de robótica
- 💬 **Contribuir** con mejoras o reportar bugs
- 📢 **Mencionar** el proyecto en tus trabajos académicos

---

<div align="center">

**Hecho con ❤️ por Mario David Alvarez y el Semillero de Robótica SIRO**

**ROS2 Humble • Ubuntu 22.04 • Nav2 • SLAM Toolbox • Gazebo**

*Transformando la educación en robótica, un commit a la vez* 🚀

[![GitHub stars](https://img.shields.io/github/stars/mrdavidavl/Axioma_robot?style=social)](https://github.com/mrdavidavl/Axioma_robot)
[![GitHub forks](https://img.shields.io/github/forks/mrdavidavl/Axioma_robot?style=social)](https://github.com/mrdavidavl/Axioma_robot/fork)
[![Follow @mrdavidavl](https://img.shields.io/github/followers/mrdavidavl?label=Follow&style=social)](https://github.com/mrdavidavl)

</div>
