# ⚙️ Sistema de Control del Robot Axioma

> No hay ningún PID en el proyecto. El plugin de Gazebo fija velocidades de junta
> y la física hace el resto. Este documento describe qué capa impone cada límite
> y, sobre todo, cuáles de los parámetros que parecen límites **no hacen nada**.

## 1. Arquitectura

```
Nav2 behavior tree
    |   NavFn                    plan global
    |   DWB                      trayectoria local
    |   velocity smoother        rampas de aceleración
    v
/cmd_vel  (geometry_msgs/Twist)
    v
gz-sim-diff-drive-system         cinemática inversa
    |
    v
4 juntas de rueda (2 pares sincronizados)
    v
Física (dartsim)                 fricción, arrastre, contacto
    v
/odom  +  odom_to_tf  ->  TF odom → base_link
    v
AMCL  ->  TF map → odom
```

---

## 2. El plugin de control

### 2.1 Cuál es

**`gz-sim-diff-drive-system`**, de Ignition Gazebo Fortress.

> ⚠️ **No** es `libgazebo_ros_diff_drive.so`. Ese es el plugin de Gazebo Classic
> y tiene otro juego de etiquetas. Confundirlos es la causa de la sección 2.3.

### 2.2 Configuración real

```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
  <left_joint>base_to_wheel1</left_joint>
  <left_joint>base_to_wheel2</left_joint>
  <right_joint>base_to_wheel4</right_joint>
  <right_joint>base_to_wheel3</right_joint>

  <wheel_separation>0.1679</wheel_separation>
  <wheel_radius>0.0381</wheel_radius>

  <min_linear_velocity>-0.26</min_linear_velocity>
  <max_linear_velocity>0.26</max_linear_velocity>
  <min_angular_velocity>-1.0</min_angular_velocity>
  <max_angular_velocity>1.0</max_angular_velocity>
  <min_linear_acceleration>-1.0</min_linear_acceleration>
  <max_linear_acceleration>1.0</max_linear_acceleration>
  <min_angular_acceleration>-3.2</min_angular_acceleration>
  <max_angular_acceleration>3.2</max_angular_acceleration>

  <odom_publish_frequency>50</odom_publish_frequency>
  <topic>cmd_vel</topic>
  <odom_topic>odom</odom_topic>
  <frame_id>odom</frame_id>
  <child_frame_id>base_link</child_frame_id>
  <tf_topic>tf</tf_topic>
</plugin>
```

### 2.3 Etiquetas que el plugin ignora

El modelo llevaba estas dos, heredadas de Gazebo Classic:

```xml
<max_wheel_torque>20</max_wheel_torque>
<max_wheel_acceleration>1</max_wheel_acceleration>
```

`gz-sim-diff-drive-system` **no las lee**, y no avisa. Consecuencia: el robot
no tenía ningún límite de aceleración mientras el DWB planificaba dando por
supuesto que sí. Se puede comprobar sobre la librería instalada:

```bash
strings /usr/lib/x86_64-linux-gnu/libignition-gazebo6-diff-drive-system.so \
  | grep -xE 'max_(wheel_)?(torque|acceleration|linear_acceleration)'
```

Solo aparecen las variantes `linear`/`angular`. Por eso la sección 2.2 usa
`max_linear_acceleration` y `max_angular_acceleration`, con los mismos valores
que Nav2.

### 2.4 Qué hace en cada ciclo

1. Lee `/cmd_vel`
2. Satura $(v, \omega)$ contra los límites de velocidad y aceleración
3. Aplica la cinemática inversa (ver `cinematica.md`, §5)
4. Escribe la velocidad objetivo en las 4 juntas
5. Integra las **posiciones reales de las juntas** para la odometría
6. Publica `/odom` a 50 Hz

El paso 5 es importante: la odometría sale de lo que las ruedas realmente han
girado, no del comando. Si una rueda patina, la odometría se lo cree, igual que
un encoder en el robot físico.

---

## 3. Dónde se impone cada límite

| Magnitud | DWB | Velocity smoother | Plugin | Física |
|----------|-----|-------------------|--------|--------|
| $v$ | 0.26 m/s | 0.26 m/s | 0.26 m/s | fricción |
| $\omega$ | 1.0 rad/s | 1.0 rad/s | 1.0 rad/s | arrastre lateral |
| $\dot v$ | 1.0 m/s² | 1.0 m/s² | 1.0 m/s² | $\mu g$ |
| $\dot\omega$ | 3.2 rad/s² | 3.2 rad/s² | 3.2 rad/s² | — |

Las tres capas de software llevan ahora el mismo valor. Cuando no coincidían, el
DWB elegía trayectorias que el robot no podía seguir y el controlador acababa
disparando comportamientos de recuperación.

**Límite por fricción** (cota superior física, muy por encima de las anteriores):

$$
a_{\mu} = \mu_2 \, g = 1.0 \times 9.81 = 9.81 \text{ m/s}^2
$$

---

## 4. Controlador local: DWB

**Plugin**: `dwb_core::DWBLocalPlanner`, a `controller_frequency: 20.0` Hz.

### 4.1 Ventana dinámica

```yaml
max_vel_x: 0.26          min_vel_x: 0.0
max_vel_theta: 1.0       max_speed_xy: 0.26
max_vel_y: 0.0           vy_samples: 1      # differential drive: sin eje y
vx_samples: 20           vtheta_samples: 20
sim_time: 1.7
acc_lim_x: 1.0           decel_lim_x: -1.0
acc_lim_theta: 3.2       decel_lim_theta: -3.2
```

`vy_samples: 1` porque el robot no tiene movilidad lateral; muestrear más
velocidades en $y$ solo gasta cómputo generando trayectorias idénticas.

### 4.2 Funciones de coste

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
          "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

BaseObstacle.scale: 0.02      PathAlign.scale: 32.0
GoalAlign.scale:   24.0       PathDist.scale:  32.0
GoalDist.scale:    24.0       RotateToGoal.scale: 32.0
```

$$
J = \sum_i w_i J_i
$$

### 4.3 Comprobador de progreso

```yaml
required_movement_radius: 0.10   # m
movement_time_allowance:  10.0   # s
```

El radio por defecto de Nav2 es 0.5 m, demasiado para un robot que va a
0.26 m/s como máximo: tardaría 2 s solo en recorrerlo. Con 0.10 m y 10 s, un
atasco dispara la recuperación en una décima parte del tiempo que con los 30 s
que tuvo este proyecto en algún momento.

---

## 5. Velocity smoother

```yaml
smoothing_frequency: 20.0
feedback: "OPEN_LOOP"
max_velocity: [ 0.26, 0.0,  1.0]
min_velocity: [-0.26, 0.0, -1.0]
max_accel:    [ 1.0,  0.0,  3.2]
max_decel:    [-1.0,  0.0, -3.2]
```

Para cada eje:

$$
v_k = \mathrm{clip}\big(v_{des},\; v_{k-1} - a_{max}\Delta t,\; v_{k-1} + a_{max}\Delta t\big),
\qquad \Delta t = 0.05 \text{ s}
$$

---

## 6. Localización: AMCL

```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
alpha1 ... alpha5: 0.05
laser_min_range: 0.12      laser_max_range: 8.0
max_beams: 120             min_particles: 1000    max_particles: 5000
update_min_d: 0.1 m        update_min_a: 0.1 rad
set_initial_pose: true     initial_pose: (0, 0, 0)
```

Los $\alpha$ son las varianzas del modelo de movimiento diferencial:

$$
\sigma_{rot1}^2 = \alpha_1 |\Delta\theta| + \alpha_2 \|\Delta \mathbf{p}\|
$$
$$
\sigma_{trans}^2 = \alpha_3 \|\Delta \mathbf{p}\| + \alpha_4 |\Delta\theta|
$$

`laser_max_range` **debe** coincidir con el alcance del sensor. Cuando decía 3.5
m con un YDLIDAR X3 PRO de 8 m, AMCL descartaba más de la mitad de los retornos.

---

## 7. Precisión medida

Toda la validación se hace contra la pose real de Gazebo, publicada por
`/world/default/dynamic_pose/info` y puenteada a ROS. No contra la odometría:
comparar la odometría consigo misma no demuestra nada.

**Recorrido de mapeo de 90 m por las cuatro salas:**

| | Media | p95 | Máximo |
|---|---|---|---|
| Error de posición, odometría cruda | 1.37 m | 3.27 m | 3.79 m |
| Error de posición, SLAM Toolbox | 0.033 m | 0.109 m | 0.141 m |
| Error de guiñada, SLAM Toolbox | 1.09° | — | 10.2° |

**Navegación sobre el mapa resultante:** 5 de 5 objetivos, cero comportamientos
de recuperación, error final entre 0.05 y 0.15 m.

### 7.1 En el robot físico

La odometría en simulación es exacta en traslación porque no hay ruido de
encoder. En el robot real hay que contar además con:

1. Deslizamiento de las ruedas: inherente al skid-steer, y depende de la
   superficie
2. Resolución de encoder: 1000 PPR
3. Dispersión del diámetro entre ruedas

La vía efectiva $W = 0.1679$ m está calibrada **para la fricción simulada**. En
el robot físico hay que repetir la calibración de `cinematica.md` §6.3 sobre la
superficie de trabajo.

---

## 8. Limitaciones

### 8.1 No hay lazo de velocidad

El plugin fija velocidades de junta directamente. En el robot real hacen falta:

- Un PID por motor
- Medida de velocidad con encoders
- Compensación de deslizamiento, o un EKF que fusione IMU y odometría

El URDF ya declara `imu_link`, pero **no hay sensor IMU en el SDF**: ese frame
está hoy sin usar.

### 8.2 Sin límite de par

El plugin no impone par. La única cota es la fricción, $F = \mu_2 m g / 4$ por
rueda. Un motor real saturará mucho antes.

---

## 9. Referencias

- Plugin: `src/axioma_gazebo/models/axioma_v2/model.sdf`
- Nav2: `src/axioma_navigation/config/nav2_params.yaml`
- [Nav2 DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Nav2 AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

**Autor**: Mario David Alvarez Vallejo
