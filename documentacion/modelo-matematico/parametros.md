# 📏 Parámetros Físicos del Robot Axioma

> Cada valor de este documento sale de un archivo del repositorio. Los que están
> **calibrados** en vez de medidos llevan la etiqueta 🎯 y una referencia a cómo
> se obtuvieron.

---

## 1. Geometría

### 1.1 Ruedas

| Parámetro | Símbolo | Valor | Fuente |
|-----------|---------|-------|--------|
| Radio (colisión y plugin) | $r$ | 0.0381 m | `model.sdf` |
| Radio del mesh visual | — | 0.0405 m | `llanta.stl` |
| Ancho de colisión | $w$ | 0.030 m | `model.sdf` |
| Desplazamiento de la colisión en $y$ local | — | +0.020 m | `model.sdf` |

> El mesh visual es 2.4 mm mayor que el cilindro de colisión. Solo afecta al
> aspecto: la física y la odometría usan 0.0381 m.

### 1.2 Chasis y vía

| Parámetro | Símbolo | Valor | Fuente |
|-----------|---------|-------|--------|
| Vía entre juntas | $W_j$ | 0.13635 m | Orígenes de junta |
| Vía de contacto | $W_c$ | 0.17635 m | $W_j + 2 \times 0.02$ |
| **Vía efectiva (plugin)** | $W$ | **0.1679 m** 🎯 | `model.sdf`, ver `cinematica.md` §6 |
| Distancia entre ejes | $L$ | 0.13564 m | Orígenes de junta |
| Largo real | — | 0.2166 m | Meshes visuales |
| Ancho real | — | 0.2220 m | Meshes visuales |
| Alto real | — | 0.1548 m | Meshes visuales |
| Radio circunscrito desde `base_link` | — | 0.1577 m | Meshes visuales |

`base_link` queda 2.58 mm por debajo del plano del suelo: el centro de rueda
está a $z = 0.04068$ y el radio es 0.0381, así que $0.0381 - 0.04068 = -0.00258$.

### 1.3 Posiciones de rueda

| Rueda | $(x, y, z)$ [m] | Joint | rpy |
|-------|-----------------|-------|-----|
| W1 (DI) | (0.06442, 0.07385, 0.04068) | `base_to_wheel1` | (π, 0, π) |
| W2 (TI) | (-0.071224, 0.07385, 0.04068) | `base_to_wheel2` | (π, 0, π) |
| W3 (TD) | (-0.071224, -0.0625, 0.04068) | `base_to_wheel3` | (0, 0, π) |
| W4 (DD) | (0.064423, -0.0625, 0.04068) | `base_to_wheel4` | (0, 0, π) |

Las rpy hacen que el eje declarado de cada junta (`0 1 0` a la izquierda,
`0 -1 0` a la derecha) apunte al mismo $+y$ de `base_link`, de modo que giro
positivo es avance en los cuatro casos.

---

## 2. Propiedades inerciales

### 2.1 Masa

| Componente | Cantidad | Masa unitaria | Total |
|------------|----------|---------------|-------|
| `base_link` (chasis) | 1 | 5.000 kg | 5.000 kg |
| Ruedas | 4 | 0.100 kg | 0.400 kg |
| LiDAR (`base_scan`) | 1 | 0.125 kg | 0.125 kg |
| **TOTAL** | | | **5.525 kg** |

### 2.2 Tensores de inercia

$$
\mathbf{I}_{base} = \mathrm{diag}(0.1135,\ 0.4260,\ 0.5205) \text{ kg·m}^2
$$
$$
\mathbf{I}_{rueda} = \mathrm{diag}(0.0029,\ 0.0029,\ 0.0056) \text{ kg·m}^2
$$
$$
\mathbf{I}_{lidar} = \mathrm{diag}(0.0010,\ 0.0010,\ 0.0010) \text{ kg·m}^2
$$

---

## 3. Parámetros dinámicos

### 3.1 Fricción rueda–suelo

```xml
<friction>
  <ode>
    <fdir1>0 0 1</fdir1>   <!-- eje del cilindro = eje de la rueda -->
    <mu>0.05</mu>          <!-- lateral (arrastre) -->
    <mu2>1.0</mu2>         <!-- rodadura (tracción) -->
    <slip1>0</slip1>
    <slip2>0</slip2>
  </ode>
</friction>
```

| Parámetro | Valor | Qué es |
|-----------|-------|--------|
| `fdir1` | (0, 0, 1) | Primera dirección de fricción, en el frame de la colisión |
| $\mu$ | 0.05 | Coeficiente **a lo largo de `fdir1`**: lateral |
| $\mu_2$ | 1.0 | Coeficiente perpendicular: rodadura |

Sin `fdir1` el motor de física elige una dirección arbitraria y $\mu$/$\mu_2$
dejan de tener el significado que uno espera. Ver `cinematica.md` §6.3.

**Aceleración máxima por fricción**:
$$
a_{\mu} = \mu_2 \, g = 9.81 \text{ m/s}^2
$$

### 3.2 Contacto (ODE)

| Parámetro | Valor |
|-----------|-------|
| `soft_cfm` | 0 |
| `soft_erp` | 0.2 |
| `kp` | $10^{13}$ |
| `kd` | 1.0 |
| `max_vel` | 0.01 m/s |
| `min_depth` | 0.01 m |

### 3.3 Motor de física

**dartsim** (por defecto en Ignition Fortress). La etiqueta `<physics type="ode">`
del mundo es heredada y gz-sim la ignora; los parámetros `<ode>` de fricción sí
se traducen.

---

## 4. Límites operacionales

### 4.1 Cinemáticos

| Parámetro | Valor | Impuesto en |
|-----------|-------|-------------|
| $v_{max}$ | 0.26 m/s | DWB, smoother, plugin |
| $\omega_{max}$ | 1.0 rad/s | DWB, smoother, plugin |
| $a_{max}$ | 1.0 m/s² | DWB, smoother, plugin |
| $\alpha_{max}$ | 3.2 rad/s² | DWB, smoother, plugin |

### 4.2 Etiquetas sin efecto

`max_wheel_torque` y `max_wheel_acceleration` son de Gazebo Classic.
`gz-sim-diff-drive-system` no las lee. Se sustituyeron por
`max_linear_acceleration` / `max_angular_acceleration`, que sí lee. Detalle en
`control.md` §2.3.

### 4.3 Juntas

```xml
<limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
```

Giro libre, sin límite de par declarado.

---

## 5. Sensor LiDAR

### 5.1 YDLIDAR X3 PRO

| Parámetro | Valor |
|-----------|-------|
| Posición | $(0, 0, 0.15)$ m sobre `base_link` |
| Frame | `base_scan` |
| Tópico | `/scan` |
| Alcance | 0.12 – 8.0 m |
| Campo | 360°, de $-\pi$ a $+\pi - \Delta$ |
| Muestras por vuelta | 571 |
| Resolución angular | 0.630° |
| Frecuencia de giro | 7 Hz (ajustable 5–10 Hz) |
| Tasa de telemetría | 4 kHz |
| Ruido | gaussiano, $\sigma = 0.015$ m |
| Rechazo de luz ambiente | 40 klux |

La resolución **no es un dato fijo del sensor**: es de canal único, así que

$$
\text{muestras por vuelta} = \frac{\text{tasa de telemetría}}{\text{frecuencia de giro}}
= \frac{4000}{7} \approx 571
$$

| Frecuencia | Muestras/vuelta | Resolución |
|------------|-----------------|------------|
| 5 Hz | 800 | 0.450° |
| **7 Hz** | **571** | **0.630°** |
| 10 Hz | 400 | 0.900° |

Si se cambia `<update_rate>` hay que cambiar `<samples>` en la misma proporción.

### 5.2 Transformada

$$
\mathbf{T}^{base}_{scan} =
\begin{bmatrix} 1&0&0&0 \\ 0&1&0&0 \\ 0&0&1&0.15 \\ 0&0&0&1 \end{bmatrix}
$$

Sin rotación: el escaneo ya viene alineado con `base_link`.

---

## 6. Navegación

### 6.1 Costmaps

| | Local | Global |
|---|---|---|
| Frecuencia de actualización | 5.0 Hz | 1.0 Hz |
| Tamaño | 6 × 6 m (rolling) | tamaño del mapa |
| Resolución | 0.05 m | 0.05 m |
| `robot_radius` | 0.16 m | 0.16 m |
| `inflation_radius` | 0.25 m | 0.30 m |
| `cost_scaling_factor` | 3.0 | 3.0 |
| Capas | obstacle, inflation | static, obstacle, inflation |
| Rango de trazado | 6.0 m | 8.0 m |

El local traza hasta 6 m porque su ventana mide 6 × 6: más allá no hay celda que
actualizar.

### 6.2 Tolerancias

```yaml
xy_goal_tolerance:  0.15 m
yaw_goal_tolerance: 0.25 rad  (14.3°)
```

---

## 7. SLAM

```yaml
resolution: 0.05
min_laser_range: 0.12          max_laser_range: 8.0
minimum_time_interval: 0.2
minimum_travel_distance: 0.2   minimum_travel_heading: 0.2

do_loop_closing: true
loop_search_maximum_distance: 2.0
loop_match_maximum_variance_coarse: 0.5
loop_match_minimum_response_fine: 0.5
loop_match_minimum_chain_size: 10
```

Los umbrales de nodo nuevo (0.2 m / 0.2 rad) deben quedar **dentro** de la
ventana de búsqueda del emparejador de scans (±0.25 m por
`correlation_search_space_dimension`, ±20° por `coarse_search_angle_offset`).
Con los 0.5/0.5 originales el emparejador no alcanzaba a cerrar la
correspondencia y el grafo divergía.

### 7.1 Mapa

| Parámetro | Valor |
|-----------|-------|
| Origen | (-7.01, -2.99) m |
| Resolución | 0.05 m/píxel |
| Umbrales | ocupado 0.65, libre 0.25 |

Producido por SLAM Toolbox recorriendo `office.world`, no derivado
analíticamente. `maps/ground_truth.pgm` sí es la planta exacta y sirve para
puntuar el mapa de SLAM con `axioma_slam/scripts/score_map.py`.

---

## 8. Cálculos derivados

### 8.1 Velocidades de rueda

$$
\omega_{recto} = \frac{v_{max}}{r} = \frac{0.26}{0.0381} = 6.82 \text{ rad/s} = 65.2 \text{ RPM}
$$
$$
\omega_{giro} = \frac{\omega_{max} W}{2r} = \frac{1.0 \times 0.1679}{2 \times 0.0381} = 2.20 \text{ rad/s} = 21.0 \text{ RPM}
$$
$$
\omega_{comb} = \frac{v_{max} + \omega_{max} W/2}{r} = 9.03 \text{ rad/s}
$$

### 8.2 Frenado y giro

$$
d_{frenado} = \frac{v_{max}^2}{2 |a_{min}|} = \frac{0.26^2}{2 \times 1.0} = 0.034 \text{ m}
$$
$$
R_{min} = 0 \text{ m}, \qquad
R\big|_{max} = \frac{v_{max}}{\omega_{max}} = 0.26 \text{ m}
$$

---

## 9. Resumen

```python
GEOMETRY = {
    'wheel_radius':      0.0381,    # m   model.sdf
    'wheel_separation':  0.1679,    # m   model.sdf, calibrada
    'contact_track':     0.17635,   # m   derivada de las juntas
    'wheel_base':        0.13564,   # m   derivada de las juntas
    'footprint':         (0.2166, 0.2220, 0.1548),   # m, meshes
    'circumscribed_r':   0.1577,    # m,  robot_radius = 0.16
    'total_mass':        5.525,     # kg
}

KINEMATICS = {
    'max_linear_velocity':      0.26,   # m/s
    'max_angular_velocity':     1.0,    # rad/s
    'max_linear_acceleration':  1.0,    # m/s^2
    'max_angular_acceleration': 3.2,    # rad/s^2
}

RATES = {
    'odom_publish':   50.0,   # Hz   model.sdf
    'controller':     20.0,   # Hz   nav2_params.yaml
    'smoother':       20.0,   # Hz   nav2_params.yaml
    'lidar':           7.0,   # Hz   model.sdf
    'local_costmap':   5.0,   # Hz   nav2_params.yaml
}
```

---

## 10. Inconsistencias

### 10.1 Vía de rueda — ✅ resuelta

La versión anterior de este documento ya señalaba que `wheel_separation`
(0.1725 m) no coincidía con la separación de las juntas (0.13635 m) y avisaba de
que «puede causar deriva en odometría durante rotaciones».

Estaba en lo cierto. Medido: el robot giraba el **65 %** de lo comandado. Se
corrigió con fricción anisótropa más una vía efectiva calibrada; el error de
guiñada quedó por debajo del 3 %. Procedimiento en `cinematica.md` §6.

### 10.2 Masa del chasis — ⚠️ abierta

- `model.sdf`: 5.0 kg ✅ (es la que usa Gazebo)
- `axioma.urdf`: 1.0 kg ⚠️

El URDF solo lo consume `robot_state_publisher`, que ignora las inercias, así
que hoy no afecta a nada. Pasaría a importar si alguna herramienta calculara
dinámica desde el URDF.

### 10.3 `imu_link` sin sensor — ⚠️ abierta

El URDF declara `imu_link` en $(0.1, 0.1, 0.12)$, fuera de la envolvente del
chasis, y el SDF no tiene ningún sensor IMU. Es un frame TF huérfano.

### 10.4 Radio del mesh de rueda — ℹ️ informativa

Visual 0.0405 m frente a colisión 0.0381 m. Solo estético.

---

## 11. Referencias cruzadas

- **Cinemática**: [cinematica.md](./cinematica.md)
- **Control**: [control.md](./control.md)
- **Diagrama**: [render_diagram.py](./render_diagram.py) genera
  `images/modelo-matematico.png` leyendo estos mismos archivos
- **Archivos fuente**:
  - `src/axioma_gazebo/models/axioma_v2/model.sdf`
  - `src/axioma_description/urdf/axioma.urdf`
  - `src/axioma_navigation/config/nav2_params.yaml`
  - `src/axioma_slam/config/slam_params.yaml`

---

**Autor**: Mario David Alvarez Vallejo
