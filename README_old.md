# Comprehensive ROS 2 Odometry Workspace

Complete multi-sensor odometry system for ROS 2 Humble featuring LIDAR, IMU/Magnetometer, and GPS odometry estimation with advanced sensor fusion capabilities.

## Overview

This workspace provides a comprehensive odometry estimation framework with three independent sensor-based odometry packages:

1. **LIDAR Odometry (FastGICP)** - High-precision point cloud registration for LIVOX HAP sensor
2. **IMU/Magnetometer Odometry** - Madgwick filter-based orientation and motion estimation  
3. **GPS Odometry** - Absolute positioning from GNSS RTK data with geodetic conversion

All packages are designed to work independently or together in a sensor fusion framework, publishing to standardized topics for integration with the ROS 2 navigation stack.

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   LIVOX HAP     │    │   IMU/MAG       │    │   GPS RTK       │
│   /livox/lidar  │    │   /mavros/imu/* │    │   /mavros/gps/* │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          ▼                      ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ LIDAR Odometry  │    │ IMU/MAG Odom    │    │ GPS Odometry    │
│ (FastGICP)      │    │ (Madgwick)      │    │ (Geodetic)      │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          ▼                      ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│/odometry/lidar  │    │/odometry/imu_mag│    │ /odometry/gps   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
          │                      │                      │
          └──────────┬───────────┴──────────────────────┘
                     ▼
          ┌─────────────────┐
          │ Sensor Fusion   │
          │ (robot_localization,
          │  kalman_filter) │
          └─────────────────┘
```

## Package Structure

```
odometry_ws/
├── src/
│   ├── fast_gicp/                     # External FastGICP library
│   ├── lidar_odometry_fastgicp/       # LIDAR odometry package
│   │   ├── src/                       # C++ implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── imu_mag_odometry/              # IMU/Magnetometer package
│   │   ├── imu_mag_odometry/          # Python implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── gps_odometry/                  # GPS odometry package
│   │   ├── gps_odometry/              # Python implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   └── complete_odometry.launch.py    # Main workspace launch file
└── README.md                          # This file
```

## Quick Start

### 1. Dependencies Installation

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    python3-pip \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-mavros-msgs

# Python dependencies
pip3 install pyproj numpy
```

### 2. Workspace Setup

```bash
# Clone or create workspace
cd ~/Desktop
git clone <repository-url> odometry_ws  # or use existing workspace
cd odometry_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Launch Complete System

```bash
# Launch all odometry nodes
ros2 launch src/complete_odometry.launch.py

# Launch individual packages
ros2 launch lidar_odometry_fastgicp lidar_odometry.launch.py
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py
ros2 launch gps_odometry gps_odometry.launch.py

# Launch with custom topics
ros2 launch src/complete_odometry.launch.py \
    lidar_topic:=/your/lidar/topic \
    imu_topic:=/your/imu/topic \
    gps_topic:=/your/gps/topic
```
│   │   ├── include/lidar_odometry_fastgicp/
│   │   │   └── lidar_odometry.hpp    # Clase principal
│   │   ├── src/
│   │   │   ├── lidar_odometry.cpp    # Implementación
│   │   │   └── lidar_odometry_node.cpp # Nodo principal
│   │   ├── launch/
│   │   │   └── lidar_odometry_fastgicp.launch.py
│   │   ├── config/
│   │   │   └── lidar_odometry_params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── imu_mag_odometry/             # Nodo de odometría IMU/Magnetómetro
│       ├── imu_mag_odometry/
│       │   ├── __init__.py
│       │   ├── imu_mag_odometry_node.py # Nodo principal
│       │   └── madgwick_filter.py    # Filtro Madgwick
│       ├── launch/
│       │   └── imu_mag_odometry.launch.py
│       ├── config/
│       │   └── imu_mag_params.yaml
│       ├── setup.py
│       ├── package.xml
│       └── README.md                 # Documentación específica
├── build/                            # Archivos de compilación
├── install/                          # Archivos instalados
├── log/                              # Logs de compilación
└── README.md
```

## 🔧 Instalación y Configuración

### Dependencias del Sistema

```bash
# Dependencias básicas
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev libpcl-dev libyaml-cpp-dev

# ROS 2 Humble (si no está instalado)
sudo apt install -y ros-humble-desktop-full
sudo apt install -y ros-humble-pcl-conversions ros-humble-sensor-msgs ros-humble-nav-msgs \
                    ros-humble-geometry-msgs ros-humble-tf2 ros-humble-tf2-ros \
                    ros-humble-tf2-geometry-msgs

# Herramientas de desarrollo
sudo apt install -y python3-rosdep2 python3-colcon-common-extensions
```

### Configuración CUDA (Opcional)

Si tienes una GPU NVIDIA compatible, puedes habilitar la aceleración CUDA:

```bash
# Verificar GPU NVIDIA
nvidia-smi

# Instalar CUDA Toolkit (versión compatible con tu GPU)
sudo apt install -y nvidia-cuda-toolkit

# Verificar instalación
nvcc --version
```

### Compilación del Workspace

```bash
# Configurar entorno ROS 2
source /opt/ros/humble/setup.bash

# Navegar al workspace
cd odometry_ws

# Instalar dependencias con rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Compilar el workspace
colcon build --symlink-install

# Configurar entorno del workspace
source install/setup.bash
```

## 🚀 Uso

### Odometría LIDAR (FastGICP)

```bash
# Terminal 1: Configurar entorno
cd odometry_ws
source install/setup.bash

# Lanzar el nodo de odometría LIDAR
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py
```

### Odometría IMU/Magnetómetro

```bash
# Terminal 2: Lanzar odometría IMU/Mag (solo orientación - recomendado)
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py

# O con integración experimental de posición/velocidad
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py enable_integration:=true

# O con soporte de barómetro para altitud
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py use_barometer:=true
```

### Lanzamiento Combinado

```bash
# Lanzar ambos sistemas de odometría simultáneamente
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py &
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py
```

### Configuración de Parámetros

Edita el archivo `src/lidar_odometry_fastgicp/config/lidar_odometry_params.yaml`:

```yaml
# Topics
input_topic: "/livox/lidar"           # Topic de entrada del LIDAR
output_topic: "/odometry/lidar"       # Topic de salida de odometría

# Frames
base_frame: "base_link"               # Frame base del robot
odom_frame: "odom"                    # Frame de odometría

# Preprocessing
voxel_size: 0.5                       # Tamaño del voxel para downsampling (metros)

# FastGICP parameters
use_cuda: false                       # Habilitar aceleración CUDA
max_iterations: 64                    # Máximo número de iteraciones
transformation_epsilon: 1.0e-6       # Criterio de convergencia de transformación
euclidean_fitness_epsilon: 1.0e-6    # Criterio de convergencia euclidiano
max_correspondence_distance: 1.0     # Distancia máxima de correspondencia (metros)
```

### Parámetros por Línea de Comandos

```bash
# Cambiar topic de entrada
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py input_topic:=/your/lidar/topic

# Habilitar CUDA
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py use_cuda:=true

# Cambiar tamaño de voxel
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py voxel_size:=0.3
```

## 📊 Topics y Servicios

### LIDAR Odometry (FastGICP)

**Topics Subscritos**:
- `/livox/lidar` (sensor_msgs/PointCloud2): Nubes de puntos del LIDAR LIVOX HAP

**Topics Publicados**:
- `/odometry/lidar` (nav_msgs/Odometry): Odometría estimada por FastGICP

### IMU/Magnetometer Odometry

**Topics Subscritos**:
- `/mavros/imu/data` (sensor_msgs/Imu): Datos de IMU (acelerómetro, giróscopo)
- `/mavros/imu/mag` (sensor_msgs/MagneticField): Datos de magnetómetro
- `/mavros/imu/atm_pressure` (sensor_msgs/FluidPressure): Datos de barómetro (opcional)

**Topics Publicados**:
- `/odometry/imu_mag` (nav_msgs/Odometry): Odometría estimada por IMU/Mag

### Topics Comunes

- `/tf` (tf2_msgs/TFMessage): Transformaciones entre frames

### Frames
- `odom` → `base_link`: Transformación de odometría acumulada

## 🏗️ Arquitectura para Expansión

Este workspace está diseñado para agregar fácilmente más módulos de odometría:

### Estructura Actual y Futura

```
odometry_ws/src/
├── fast_gicp/                        # ✅ LIDAR FastGICP
├── lidar_odometry_fastgicp/          # ✅ Nodo LIDAR
├── imu_mag_odometry/                 # ✅ Nodo IMU/Magnetómetro
├── visual_odometry_orb/              # 🔮 Futura odometría visual
├── gps_odometry/                     # 🔮 Odometría GPS
└── sensor_fusion/                    # 🔮 Fusión de todos los sensores
```

### Topics Estándar para Fusión

```yaml
# Odometrías individuales
/odometry/lidar                       # nav_msgs/Odometry
/odometry/visual                      # nav_msgs/Odometry  
/odometry/imu                         # nav_msgs/Odometry
/odometry/gps                         # nav_msgs/Odometry

# Odometría fusionada
/odometry/fused                       # nav_msgs/Odometry

# Datos de sensores
/livox/lidar                          # sensor_msgs/PointCloud2
/camera/image_raw                     # sensor_msgs/Image
/imu/data                             # sensor_msgs/Imu
/gps/fix                              # sensor_msgs/NavSatFix
```

## 🛠️ Desarrollo

### Agregar Nuevo Módulo de Odometría

1. **Crear nuevo paquete**:
```bash
cd src
ros2 pkg create your_odometry_module --build-type ament_cmake
```

2. **Estructura recomendada**:
```cpp
// Header típico para módulo de odometría
class YourOdometry : public rclcpp::Node 
{
private:
    rclcpp::Subscription<sensor_msgs::msg::YourSensor>::SharedPtr sensor_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    // ... tu implementación
};
```

3. **Topics estándar**:
   - Entrada: `/sensor/data`
   - Salida: `/odometry/your_sensor`

### Debugging

```bash
# Ver topics activos
ros2 topic list

# Monitorear odometría
ros2 topic echo /odometry/lidar

# Ver transformaciones
ros2 run tf2_tools view_frames

# Verificar frecuencia de publicación
ros2 topic hz /odometry/lidar
```

## 📋 Troubleshooting

### Problemas Comunes

**Error de compilación con FastGICP**:
```bash
# Verificar dependencias PCL
sudo apt install libpcl-dev

# Limpiar y recompilar
rm -rf build install log
colcon build --symlink-install
```

**No se reciben datos del LIDAR**:
```bash
# Verificar topic del LIDAR
ros2 topic list | grep livox
ros2 topic echo /livox/lidar --once

# Cambiar topic en el launch
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py input_topic:=/your/actual/topic
```

**Problemas de CUDA**:
```bash
# Verificar instalación CUDA
nvidia-smi
nvcc --version

# Deshabilitar CUDA si hay problemas
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py use_cuda:=false
```

## 📈 Rendimiento

### Configuración Recomendada por Hardware

**CPU básica**:
```yaml
voxel_size: 0.5
max_iterations: 32
use_cuda: false
```

**CPU potente**:
```yaml
voxel_size: 0.3
max_iterations: 64
use_cuda: false
```

**GPU NVIDIA**:
```yaml
voxel_size: 0.2
max_iterations: 64
use_cuda: true
```

## 🤝 Contribución

Para contribuir al proyecto:

1. Fork el repositorio
2. Crea una rama para tu feature: `git checkout -b feature/nueva-funcionalidad`
3. Commit tus cambios: `git commit -am 'Añadir nueva funcionalidad'`
4. Push a la rama: `git push origin feature/nueva-funcionalidad`
5. Crea un Pull Request

## 📄 Licencia

Este proyecto está bajo la licencia MIT. Ver el archivo LICENSE para más detalles.

## 🔗 Referencias

- [FastGICP Original Repository](https://github.com/SMRT-AIST/fast_gicp)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [PCL Library](https://pointclouds.org/)
- [LIVOX SDK](https://github.com/Livox-SDK/Livox-SDK2)

Para habilitar aceleración GPU:

```bash
# Verificar GPU NVIDIA disponible
nvidia-smi

# Si hay GPU disponible, instalar CUDA toolkit
sudo apt install -y nvidia-cuda-toolkit
```

### Compilación del Workspace

```bash
# Clonar el repositorio
git clone https://github.com/fjgarco/odometry_ws.git
cd odometry_ws

# Configurar entorno ROS 2
source /opt/ros/humble/setup.bash

# Instalar dependencias con rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --symlink-install

# Configurar entorno del workspace
source install/setup.bash
```

## 🚀 Uso

### Lanzamiento Básico

```bash
# Configurar entorno
source /opt/ros/humble/setup.bash
source install/setup.bash

# Lanzar odometría LIDAR
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py
```

### Configuración de Parámetros

El archivo de configuración se encuentra en `src/lidar_odometry_fastgicp/config/lidar_odometry_params.yaml`:

```yaml
# Topics
input_topic: "/livox/lidar"      # Topic de entrada del LIDAR
output_topic: "/odometry/lidar"  # Topic de salida de odometría

# Frames
base_frame: "base_link"          # Frame del robot
odom_frame: "odom"              # Frame de odometría

# Preprocessing
voxel_size: 0.5                 # Tamaño del voxel para downsampling (metros)

# FastGICP
use_cuda: false                 # Habilitar aceleración CUDA
max_iterations: 64              # Máximo número de iteraciones
transformation_epsilon: 1.0e-6  # Umbral de convergencia
euclidean_fitness_epsilon: 1.0e-6
max_correspondence_distance: 1.0 # Distancia máxima de correspondencia (metros)
```

### Lanzamiento con Parámetros Personalizados

```bash
# Usar otro topic de entrada
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py \
    input_topic:=/custom/lidar \
    use_cuda:=true

# Usar archivo de configuración personalizado
ros2 launch lidar_odometry_fastgicp lidar_odometry_fastgicp.launch.py \
    config_file:=/path/to/custom_params.yaml
```

## 📊 Topics y Mensajes

### Suscripciones

- `/livox/lidar` (`sensor_msgs/PointCloud2`): Nubes de puntos del LIDAR LIVOX

### Publicaciones

- `/odometry/lidar` (`nav_msgs/Odometry`): Odometría estimada
- `/tf` (`tf2_msgs/TFMessage`): Transformaciones entre frames

### Frames

- `odom` → `base_link`: Transformación de odometría

## ⚙️ Configuración del Sensor LIVOX HAP

Para usar con el sensor LIVOX HAP, asegúrate de que esté publicando en el topic correcto:

```bash
# Verificar topics disponibles
ros2 topic list

# Ver datos del LIDAR
ros2 topic echo /livox/lidar --max-count 1

# Verificar frecuencia
ros2 topic hz /livox/lidar
```

## 🔧 Solución de Problemas

### Error de Compilación

Si encuentras errores de compilación:

```bash
# Limpiar build
rm -rf build install log

# Instalar dependencias faltantes
rosdep install --from-paths src --ignore-src -r -y

# Recompilar
colcon build --symlink-install
```

### Problemas de CUDA

Si CUDA no está disponible:

```bash
# Verificar configuración
nvidia-smi

# Desactivar CUDA en configuración
# En lidar_odometry_params.yaml:
use_cuda: false
```

### Sin Datos de LIDAR

Si no se reciben datos:

```bash
# Verificar conexión del sensor
ros2 topic list | grep lidar

# Verificar configuración de red (si es Ethernet)
ping <IP_DEL_SENSOR>

# Verificar topic de entrada en configuración
```

## 🚧 Roadmap - Próximas Funcionalidades

### 1. Odometría Visual
- [ ] Integración con cámaras estéreo/monoculares
- [ ] Algoritmos VO/VIO (ORB-SLAM3, OpenVINS)
- [ ] Fusión LIDAR-Visual

### 2. Sensores Inerciales
- [ ] Integración IMU
- [ ] Magnetómetro para corrección de heading
- [ ] Pre-integración inercial

### 3. GPS/GNSS
- [ ] Integración RTK-GPS
- [ ] Georeferenciación global
- [ ] Switch automático indoor/outdoor

### 4. Fusión de Sensores
- [ ] Extended Kalman Filter (EKF)
- [ ] Graph-based SLAM
- [ ] Loop closure detection

### 5. Utilidades
- [ ] Interfaz de calibración
- [ ] Herramientas de evaluación
- [ ] Grabación y reproducción de datos

## 📁 Estructura para Expansión

El workspace está diseñado para añadir fácilmente nuevos módulos:

```
odometry_ws/src/
├── fast_gicp/                    # ✅ LIDAR registration
├── lidar_odometry_fastgicp/      # ✅ LIDAR odometry
├── visual_odometry/              # 🚧 Próximo: Visual odometry
├── imu_integration/              # 🚧 Próximo: IMU processing
├── gps_integration/              # 🚧 Próximo: GPS processing
├── sensor_fusion/                # 🚧 Próximo: Multi-sensor fusion
└── odometry_evaluation/          # 🚧 Próximo: Evaluation tools
```

## 🤝 Contribución

Para añadir nuevas funcionalidades:

1. Crear un nuevo paquete en `src/`
2. Seguir las convenciones de naming
3. Actualizar este README
4. Añadir tests apropiados

## 📄 Licencia

MIT License - Ver archivo LICENSE para detalles

## 📧 Contacto

Para soporte o preguntas, crear un issue en el repositorio GitHub.

---

**Estado del Proyecto**: 🟡 En desarrollo activo  
**Última actualización**: Agosto 2025
