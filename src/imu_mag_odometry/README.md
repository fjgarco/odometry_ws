# IMU and Magnetometer Odometry Package

Este paquete ROS 2 proporciona estimación de odometría basada en IMU (Unidad de Medición Inercial) y magnetómetro, con soporte opcional para barómetro. Se enfoca principalmente en la estimación precisa de orientación y opcionalmente puede integrar aceleraciones para estimar velocidad y posición.

## 🎯 Funcionalidades

- **Orientación precisa**: Filtro Madgwick para fusión IMU/magnetómetro
- **Integración opcional**: Estimación de velocidad y posición por integración de aceleraciones
- **Soporte de barómetro**: Estimación de altitud usando datos de presión
- **Configuración flexible**: Múltiples parámetros configurables
- **Preparado para fusión**: Arquitectura lista para combinar con otros sensores

## ⚠️ Limitaciones Importantes

1. **Deriva de posición**: La integración de aceleraciones deriva rápidamente sin corrección externa
2. **Precisión del magnetómetro**: Sensible a interferencias magnéticas locales
3. **Calibración requerida**: IMU y magnetómetro deben estar calibrados correctamente
4. **Solo orientación confiable**: Para posición precisa, combine con GPS, encoders o visión

## 📊 Topics y Mensajes

### Subscripciones

| Topic | Tipo de Mensaje | Descripción |
|-------|-----------------|-------------|
| `/mavros/imu/data` | `sensor_msgs/Imu` | Datos de IMU (acelerómetro, giróscopo, orientación) |
| `/mavros/imu/mag` | `sensor_msgs/MagneticField` | Datos de magnetómetro |
| `/mavros/imu/atm_pressure` | `sensor_msgs/FluidPressure` | Datos de presión atmosférica (opcional) |

### Publicaciones

| Topic | Tipo de Mensaje | Descripción |
|-------|-----------------|-------------|
| `/odometry/imu_mag` | `nav_msgs/Odometry` | Odometría estimada (orientación + posición/velocidad opcional) |
| `/tf` | `geometry_msgs/TransformStamped` | Transformadas TF (opcional) |

## 🔧 Configuración

### Parámetros Principales

```yaml
# Filtro de orientación
orientation_filter: "madgwick"     # "madgwick" o "use_imu_orientation"
madgwick_beta: 0.1                 # Ganancia del filtro (0.01-1.0)

# Integración de aceleraciones (¡ADVERTENCIA: deriva rápido!)
enable_integration: false          # Habilitar integración para posición/velocidad
gravity_compensation: true         # Compensar gravedad en mediciones

# Barómetro
use_barometer: false              # Usar barómetro para altitud
```

### Configuración de Covarianzas

```yaml
orientation_covariance: 0.01      # Incertidumbre de orientación (rad²)
position_covariance: 1.0          # Incertidumbre de posición (m²)
velocity_covariance: 0.1          # Incertidumbre de velocidad (m²/s²)
```

## 🚀 Uso

### Lanzamiento Básico (Solo Orientación)

```bash
# Solo orientación - RECOMENDADO para uso general
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py
```

### Lanzamiento con Integración (Experimental)

```bash
# Con integración de posición/velocidad - EXPERIMENTAL
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py \
    enable_integration:=true
```

### Lanzamiento con Barómetro

```bash
# Con estimación de altitud por barómetro
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py \
    use_barometer:=true
```

### Configuración Personalizada

```bash
# Configuración completa personalizada
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py \
    imu_topic:=/your/imu/topic \
    mag_topic:=/your/mag/topic \
    output_topic:=/your/odometry/topic \
    orientation_filter:=madgwick \
    madgwick_beta:=0.05 \
    enable_integration:=false \
    use_barometer:=true
```

## 📈 Filtros de Orientación

### Filtro Madgwick (Recomendado)

- **Descripción**: Algoritmo de gradiente descendente para fusión IMU/MARG
- **Ventajas**: Computacionalmente eficiente, buena precisión
- **Parámetro beta**: 
  - Bajo (0.01-0.05): Más estable, convergencia lenta
  - Alto (0.1-0.5): Convergencia rápida, menos estable

### Usar Orientación de IMU

- **Descripción**: Utiliza directamente la orientación del mensaje IMU
- **Cuándo usar**: Si la IMU ya realiza fusión interna confiable
- **Ventajas**: Sin procesamiento adicional

## 🔬 Modos de Operación

### Modo 1: Solo Orientación (Recomendado)

```yaml
enable_integration: false
use_barometer: false
```

**Salida confiable**:
- ✅ Orientación (roll, pitch, yaw)
- ❌ Posición (siempre 0,0,0)
- ❌ Velocidad (siempre 0,0,0)

### Modo 2: Orientación + Altitud Barométrica

```yaml
enable_integration: false
use_barometer: true
```

**Salida**:
- ✅ Orientación (roll, pitch, yaw)
- ✅ Altitud relativa (Z)
- ❌ Posición XY (siempre 0,0)
- ❌ Velocidad (siempre 0,0,0)

### Modo 3: Integración Completa (Experimental)

```yaml
enable_integration: true
use_barometer: true  # opcional
```

**Salida (⚠️ deriva rápidamente)**:
- ✅ Orientación (roll, pitch, yaw)
- ⚠️ Posición (X, Y, Z) - deriva sin corrección
- ⚠️ Velocidad (vx, vy, vz) - deriva sin corrección

## 🛠️ Desarrollo y Extensión

### Estructura del Código

```
imu_mag_odometry/
├── imu_mag_odometry/
│   ├── __init__.py
│   ├── imu_mag_odometry_node.py      # Nodo principal
│   └── madgwick_filter.py            # Implementación del filtro
├── launch/
│   └── imu_mag_odometry.launch.py    # Archivo de lanzamiento
├── config/
│   └── imu_mag_params.yaml           # Configuración por defecto
└── README.md
```

### Agregar Nuevos Filtros

Para añadir un nuevo filtro de orientación:

1. Crear nueva clase de filtro en un archivo separado
2. Modificar `imu_mag_odometry_node.py`:
```python
# En declare_parameters()
self.declare_parameter('orientation_filter', 'nuevo_filtro')

# En imu_callback()
elif self.orientation_filter == 'nuevo_filtro':
    # Implementar nueva lógica
```

### Integración con Otros Sensores

Para preparar fusión con encoders u otros sensores:

```python
# Ejemplo de estructura para fusión futura
class MultiSensorFusion:
    def __init__(self):
        self.imu_odometry = ImuMagOdometry()
        self.encoder_odometry = None  # Futuro
        self.visual_odometry = None   # Futuro
        
    def fuse_measurements(self):
        # Kalman filter o EKF para fusión
        pass
```

## 📊 Calibración y Debugging

### Verificar Datos de Entrada

```bash
# Verificar datos de IMU
ros2 topic echo /mavros/imu/data --once

# Verificar magnetómetro
ros2 topic echo /mavros/imu/mag --once

# Verificar barómetro
ros2 topic echo /mavros/imu/atm_pressure --once
```

### Monitorear Salida

```bash
# Monitorear odometría
ros2 topic echo /odometry/imu_mag

# Verificar frecuencia
ros2 topic hz /odometry/imu_mag

# Visualizar transformadas
ros2 run tf2_tools view_frames
```

### Calibración del Magnetómetro

El magnetómetro requiere calibración para compensar:
- **Hard iron bias**: Offset constante
- **Soft iron distortion**: Distorsión de escala

Utilice herramientas como `mavros` para calibración:
```bash
# Ejemplo con MAVROS
rosrun mavros mavcmd calib mag
```

## 🔄 Integración con el Workspace Odometry

Este paquete está diseñado para integrarse perfectamente con otros módulos de odometría:

```yaml
# Topics estándar del workspace
/odometry/lidar        # Desde lidar_odometry_fastgicp
/odometry/imu_mag      # Desde este paquete
/odometry/visual       # Futuro paquete visual
/odometry/gps          # Futuro paquete GPS
/odometry/fused        # Futuro paquete de fusión
```

## 📚 Referencias Técnicas

- [Madgwick Filter Paper](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html)
- [IMU Calibration Guide](https://docs.px4.io/main/en/config/compass.html)
- [ROS 2 nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)

## 🚨 Advertencias de Uso

1. **NO usar integración para navegación crítica** - Solo para experimentos
2. **Calibrar magnetómetro regularmente** - Especialmente tras cambios de ubicación
3. **Considerar interferencias magnéticas** - Alejar de motores y metales
4. **Validar orientación con referencia externa** - GPS compass o visual odometry
