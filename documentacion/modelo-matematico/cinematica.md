# 🔄 Cinemática del Robot Axioma 4WD

> Todos los parámetros de este documento se leen de `model.sdf`, `axioma.urdf` y
> `nav2_params.yaml`. El diagrama de `images/modelo-matematico.png` se genera con
> `render_diagram.py`, que lee esos mismos archivos, así que no puede quedar
> desfasado respecto a este texto.

## 1. Introducción

El Axioma es un **skid-steer de 4 ruedas fijas**: no hay dirección, se gira
haciendo girar los dos lados a distinta velocidad. Se controla como un
differential drive:

- Las ruedas izquierdas (W1 delantera, W2 trasera) giran solidarias
- Las ruedas derechas (W3 trasera, W4 delantera) giran solidarias

La diferencia entre lados produce la rotación. La sección 6 explica por qué esa
equivalencia con un differential drive **no es exacta** y qué hay que hacer para
que la odometría no se vaya.

---

## 2. Geometría

### 2.1 Posición de las ruedas

Orígenes de junta en el frame `base_link` (`axioma.urdf`, idénticos en
`model.sdf`):

| Rueda | Nombre | $(x, y, z)$ [m] | Joint |
|-------|--------|-----------------|-------|
| W1 | Delantera izquierda | (0.06442, 0.07385, 0.04068) | `base_to_wheel1` |
| W2 | Trasera izquierda | (-0.071224, 0.07385, 0.04068) | `base_to_wheel2` |
| W3 | Trasera derecha | (-0.071224, -0.0625, 0.04068) | `base_to_wheel3` |
| W4 | Delantera derecha | (0.064423, -0.0625, 0.04068) | `base_to_wheel4` |

El robot **no es simétrico** respecto a `base_link`: las ruedas izquierdas están
a $+0.07385$ m y las derechas a $-0.0625$ m. El eje de rotación real queda
$5.7$ mm a la izquierda del origen del frame. Es despreciable frente a la
resolución de los mapas (50 mm), pero conviene saberlo.

### 2.2 Tres anchos de vía distintos

Este es el punto donde es fácil equivocarse, porque hay **tres** números y solo
uno sirve para las ecuaciones:

| Concepto | Símbolo | Valor | Qué es |
|----------|---------|-------|--------|
| Vía entre juntas | $W_j$ | 0.13635 m | $y_{izq} - y_{der}$ de los orígenes de junta |
| Vía de contacto | $W_c$ | 0.17635 m | Idem, pero en el plano de contacto del cilindro de colisión, desplazado $+0.02$ m en $y$ local |
| **Vía efectiva** | $W$ | **0.1679 m** | La que va en las ecuaciones; calibrada, ver sección 6 |

$$
W_j = 0.07385 - (-0.0625) = 0.13635 \text{ m}
$$
$$
W_c = W_j + 2 \times 0.02 = 0.17635 \text{ m}
$$

**Distancia entre ejes** (wheelbase):
$$
L = 0.06442 - (-0.071224) = 0.13564 \text{ m}
$$

### 2.3 Huella real

Medida sobre los meshes visuales (`chasis.stl` + 4 × `llanta.stl` colocadas en
sus juntas):

$$
\text{largo} = 0.2166 \text{ m}, \qquad
\text{ancho} = 0.2220 \text{ m}, \qquad
\text{alto} = 0.1548 \text{ m}
$$

El radio circunscrito desde `base_link` es **0.1577 m**, que es lo que fija
`robot_radius: 0.16` en los costmaps.

### 2.4 Sistema de coordenadas

```
        y (izquierda)
        ↑
        |     W2 ●━━━━━● W1        x adelante
        |        |     |           y izquierda
        |        | {R} |           z arriba
        |        |     |           θ antihorario
        |     W3 ●━━━━━● W4
        └─────────────────→ x (adelante)
```

El LiDAR está en $(0, 0, 0.15)$: exactamente **encima** del origen de
`base_link`, sin desplazamiento lateral ni longitudinal.

### 2.5 `base_footprint`

La raíz del URDF es `base_footprint`, la proyección de `base_link` sobre el
suelo. Existe por dos motivos:

1. **KDL no admite inercia en el link raíz.** Con `base_link` de raíz,
   `robot_state_publisher` avisaba en cada arranque. `base_footprint` no lleva
   `<inertial>`, así que el aviso desaparece.
2. **Es el frame honesto para la odometría.** El plugin publica odometría plana
   con $z = 0$, que es el suelo, no el origen de `base_link`.

$$
\mathbf{T}^{footprint}_{base} = \text{traslación}(0,\ 0,\ -0.00258)
$$

Los 2.58 mm no son arbitrarios: el centro de rueda está a $z = 0.04068$ y el
radio es $0.0381$, así que con las ruedas apoyadas `base_link` queda
$0.0381 - 0.04068 = -0.00258$ m **por debajo** del suelo. Gazebo reporta
exactamente esa $z$ para el modelo.

Árbol resultante:

```
map ──(AMCL)──► odom ──(odom_to_tf)──► base_footprint ──(URDF)──► base_link
                                                                     ├─► base_scan
                                                                     ├─► imu_link
                                                                     └─► wheel_1..4
```

`robot_base_frame` en Nav2 y `base_frame` en SLAM Toolbox apuntan a
`base_footprint`. `imu_link` es la excepción del árbol: existe como reserva de
diseño y hoy no lo publica ningún sensor (ver `control.md` §8.1).

---

## 3. Modelo differential drive

### 3.1 Twist del cuerpo

$$
\mathbf{v}_R = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Un differential drive **no tiene movilidad lateral**: $v_y = 0$. Por eso
`vy_samples` está en 1 y `max_vel_y` en 0 en la configuración del DWB.

### 3.2 Agrupación de ruedas

$$
\omega_L = \omega_{W1} = \omega_{W2}, \qquad
\omega_R = \omega_{W3} = \omega_{W4}
$$

---

## 4. Cinemática directa

$$
v = \frac{r(\omega_R + \omega_L)}{2}, \qquad
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

**Derivación**. La velocidad lineal es la media de los dos lados:

$$
v = \frac{v_R + v_L}{2} = \frac{r\omega_R + r\omega_L}{2}
$$

y la angular sale de la diferencia repartida sobre la vía:

$$
\omega = \frac{v_R - v_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

**Forma matricial**, con $r = 0.0381$ m y $W = 0.1679$ m:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix} r/2 & r/2 \\ -r/W & r/W \end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix} 0.01905 & 0.01905 \\ -0.22692 & 0.22692 \end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

---

## 5. Cinemática inversa

$$
\omega_L = \frac{v - \omega W/2}{r}, \qquad
\omega_R = \frac{v + \omega W/2}{r}
$$

**Forma matricial**:

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix} 1/r & -W/(2r) \\ 1/r & W/(2r) \end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix} 26.2467 & -2.20341 \\ 26.2467 & 2.20341 \end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

---

## 6. Por qué $W$ no es una distancia medible

### 6.1 El problema

Las ecuaciones anteriores describen un differential drive ideal: dos ruedas que
solo ruedan. Un skid-steer de cuatro ruedas fijas **no puede girar sin arrastrar
las ruedas lateralmente**, y ese arrastre se opone a la guiñada. El resultado es
que el cuerpo gira **menos** de lo que predice el modelo ideal.

El plugin usa la misma $W$ para dos cosas:

1. Convertir $(v, \omega)$ del `/cmd_vel` en velocidades de rueda
2. Integrar las velocidades de rueda para publicar `/odom`

Si $W$ no coincide con la vía **efectiva** del vehículo, la odometría publica un
giro que el robot no ha hecho. Y a diferencia del error de traslación, este
error rota todo el marco: se acumula sin límite.

### 6.2 Cuánto es

Midiendo contra la pose real de Gazebo, con la vía geométrica original
($W = 0.1725$ m) y fricción isótropa:

| $\omega$ comandada | Giro real | Giro según odometría | Ratio |
|--------------------|-----------|----------------------|-------|
| 0.30 rad/s | 56.4° | 86.0° | 0.656 |
| 0.50 rad/s | 115.6° | 172.4° | 0.670 |
| 0.80 rad/s | 106.4° | 172.1° | 0.618 |
| 1.00 rad/s | 101.9° | 172.4° | 0.591 |

El robot giraba en torno al **65 %** de lo comandado, y el ratio además
**dependía de la velocidad**. En línea recta la odometría era exacta.

### 6.3 Las dos correcciones

**a) Fricción anisótropa.** En `model.sdf` las ruedas tenían $\mu = \mu_2 = 1$.
Con esa fricción isótropa el arrastre lateral resiste tanto como la rodadura.
La corrección es indicar a ODE/DART la dirección de fricción:

```xml
<fdir1>0 0 1</fdir1>   <!-- eje del cilindro = eje de la rueda -->
<mu>0.05</mu>          <!-- lateral: arrastre casi libre -->
<mu2>1.0</mu2>         <!-- rodadura: tracción intacta -->
```

`fdir1` apunta al **eje de la rueda**, que es la única dirección del cuerpo que
no rota cuando la rueda gira. Así $\mu$ queda como coeficiente lateral y
$\mu_2$ como coeficiente de rodadura.

> Bajar $\mu$ o $\mu_2$ **sin** declarar `fdir1` no sirve: el motor de física
> elige una dirección arbitraria y lo único que se consigue es perder tracción.

**b) Vía efectiva calibrada.** Con la fricción corregida queda un sesgo
residual, y se elimina ajustando $W$:

$$
W = \frac{W_c}{k}, \qquad k = \frac{\omega_{real}}{\omega_{cmd}}\bigg|_{W = W_c} = 1.05
$$
$$
W = \frac{0.17635}{1.05} = 0.1679 \text{ m}
$$

### 6.4 Resultado

| $\omega$ comandada | Ratio antes | Ratio ahora |
|--------------------|-------------|-------------|
| 0.10 rad/s | 1.208 | 0.992 |
| 0.20 rad/s | 1.082 | 0.992 |
| 0.30 rad/s | 1.040 | 1.015 |
| 0.50 rad/s | 1.064 | 0.973 |
| 0.80 rad/s | 1.092 | 1.003 |
| 1.00 rad/s | 1.026 | 0.994 |

Error de guiñada **por debajo del 3 %** en todo el rango, ya sin dependencia de
la velocidad, y traslación exacta a cualquier velocidad.

> Nota para el robot físico: exactamente el mismo razonamiento aplica. La vía
> efectiva de un skid-steer depende de la superficie, así que hay que calibrarla
> haciendo girar el robot un número conocido de vueltas y comparando con la
> odometría.

---

## 7. Odometría

### 7.1 Integración de la pose

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix} v\cos\theta \\ v\sin\theta \\ \omega \end{bmatrix}
$$

### 7.2 Integración numérica

Con $\Delta t = 0.02$ s (`odom_publish_frequency` = 50 Hz):

$$
\begin{aligned}
x_{k+1} &= x_k + v\cos\theta_k \, \Delta t \\
y_{k+1} &= y_k + v\sin\theta_k \, \Delta t \\
\theta_{k+1} &= \theta_k + \omega \, \Delta t
\end{aligned}
$$

### 7.3 Implementación

**Plugin**: `gz-sim-diff-drive-system` (Ignition Fortress). No es
`libgazebo_ros_diff_drive.so`: ese es el de Gazebo Classic y no se usa aquí.

```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
  <left_joint>base_to_wheel1</left_joint>
  <left_joint>base_to_wheel2</left_joint>
  <right_joint>base_to_wheel4</right_joint>
  <right_joint>base_to_wheel3</right_joint>
  <wheel_separation>0.1679</wheel_separation>
  <wheel_radius>0.0381</wheel_radius>
  <odom_publish_frequency>50</odom_publish_frequency>
  <topic>cmd_vel</topic>
  <odom_topic>odom</odom_topic>
  <frame_id>odom</frame_id>
  <child_frame_id>base_link</child_frame_id>
</plugin>
```

El plugin integra desde las **posiciones reales de las juntas**, no desde el
comando. Por eso, si una rueda patina, la odometría se lo cree: es el mismo
comportamiento que un encoder en el robot real.

### 7.4 TF

El plugin publica en el tópico Gz `tf`, que **no** se puentea a ROS. En su lugar
el nodo `odom_to_tf` (`axioma_gazebo`) se suscribe a `/odom` y publica la
transformada `odom → base_footprint`, forzando los nombres canónicos de frame.

El hijo es `base_footprint` y no `base_link` porque `base_link` ya cuelga de
`base_footprint` por la junta fija del URDF, y **un frame solo puede tener un
padre**. Publicar `odom → base_link` aquí competiría con
`robot_state_publisher` por él.

---

## 8. Restricciones y límites

### 8.1 Límites del cuerpo

Los mismos en Nav2 y en el plugin:

$$
|v| \leq 0.26 \text{ m/s}, \quad
|\omega| \leq 1.0 \text{ rad/s}, \quad
|\dot v| \leq 1.0 \text{ m/s}^2, \quad
|\dot\omega| \leq 3.2 \text{ rad/s}^2
$$

### 8.2 Límites de rueda

Sustituyendo en la cinemática inversa:

| Caso | Cálculo | Resultado |
|------|---------|-----------|
| Recto | $v_{max}/r$ | 6.82 rad/s (65.2 RPM) |
| Giro puro | $\omega_{max} W/(2r)$ | 2.20 rad/s (21.0 RPM) |
| Combinado | $(v_{max} + \omega_{max}W/2)/r$ | 9.03 rad/s |

### 8.3 Radio de giro

$$
R_{min} = 0 \text{ m (giro sobre el eje)}, \qquad
R\big|_{v_{max},\,\omega_{max}} = \frac{0.26}{1.0} = 0.26 \text{ m}
$$

### 8.4 Distancia de frenado

$$
d = \frac{v_{max}^2}{2|a_{min}|} = \frac{0.26^2}{2 \times 1.0} = 0.034 \text{ m}
$$

---

## 9. Implementación de referencia

```python
R = 0.0381     # wheel_radius   (model.sdf)
W = 0.1679     # wheel_separation efectiva (model.sdf, calibrada)
V_MAX, W_MAX = 0.26, 1.0


def inverse_kinematics(v, omega):
    """(v, omega) del cuerpo -> velocidades angulares de rueda [rad/s]."""
    v = max(-V_MAX, min(V_MAX, v))
    omega = max(-W_MAX, min(W_MAX, omega))
    return (v - omega * W / 2) / R, (v + omega * W / 2) / R


def forward_kinematics(w_left, w_right):
    """Velocidades angulares de rueda -> (v, omega) del cuerpo."""
    return R * (w_right + w_left) / 2, R * (w_right - w_left) / W
```

---

## 10. Referencias

- Plugin: `src/axioma_gazebo/models/axioma_v2/model.sdf`
- URDF: `src/axioma_description/urdf/axioma.urdf`
- Límites Nav2: `src/axioma_navigation/config/nav2_params.yaml`
- Diagrama: `documentacion/modelo-matematico/render_diagram.py`
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- [Gz Sim DiffDrive](https://gazebosim.org/api/sim/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html)

---

**Autor**: Mario David Alvarez Vallejo
