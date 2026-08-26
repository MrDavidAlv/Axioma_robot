# 📐 Modelo Matemático del Robot Axioma 4WD

Modelo del Axioma, una plataforma móvil de **4 ruedas motrices en configuración
skid-steer**. Cubre cinemática, control y parámetros físicos.

> **Todos los valores salen del código.** Los que están calibrados en vez de
> medidos se marcan como tales y se explica cómo se obtuvieron. El diagrama
> `images/modelo-matematico.png` lo genera [`render_diagram.py`](./render_diagram.py),
> que **lee los archivos de configuración en el momento de dibujar**, así que no
> puede quedar desfasado:
>
> ```bash
> python3 documentacion/modelo-matematico/render_diagram.py
> ```

<div align="center">
<img src="../../images/modelo-matematico.png" width="900"/>
</div>

---

## 📑 Contenido

1. **[Cinemática](./cinematica.md)** — geometría, cinemática directa e inversa,
   odometría, y **por qué la vía de rueda del modelo no es una distancia que se
   pueda medir con un calibre**
2. **[Control](./control.md)** — plugin de Gazebo, dónde se impone cada límite,
   qué parámetros son decorativos, Nav2 y AMCL
3. **[Parámetros](./parametros.md)** — tabla de referencia completa con fuentes
4. **[Diagrama](./modelo-axioma.excalidraw)** — fuente editable en Excalidraw

---

## Notación

### Sistemas de coordenadas

| Símbolo | Descripción | Frame ROS 2 |
|---------|-------------|-------------|
| $\{W\}$ | Mundo | `map` / `odom` |
| $\{R\}$ | Robot | `base_link` |

### Variables de estado

| Variable | Descripción | Unidad |
|----------|-------------|--------|
| $q = [x, y, \theta]^T$ | Pose en $\{W\}$ | m, m, rad |
| $\dot q = [v, \omega]^T$ | Twist del cuerpo | m/s, rad/s |
| $v_L, v_R$ | Velocidad lineal de cada lado | m/s |
| $\omega_L, \omega_R$ | Velocidad angular de rueda | rad/s |

---

## Ecuaciones fundamentales

### Cinemática directa

$$
v = \frac{r(\omega_R + \omega_L)}{2}, \qquad
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

### Cinemática inversa

$$
\omega_L = \frac{v - \omega W/2}{r}, \qquad
\omega_R = \frac{v + \omega W/2}{r}
$$

con $r = 0.0381$ m y $W = 0.1679$ m.

### ⚠️ Sobre $W$

$W$ **no** es la separación entre las ruedas. Hay tres anchos de vía en este
robot y solo uno funciona en las ecuaciones:

| | Valor | Qué es |
|---|---|---|
| Vía entre juntas $W_j$ | 0.13635 m | Lo que separa los orígenes de junta |
| Vía de contacto $W_c$ | 0.17635 m | Lo que separa las huellas de contacto |
| **Vía efectiva $W$** | **0.1679 m** | La que va en las ecuaciones |

Un skid-steer de cuatro ruedas fijas no puede girar sin arrastrarlas
lateralmente, y ese arrastre se opone a la guiñada. Con la vía geométrica el
robot giraba el **65 %** de lo que la odometría creía, y el error dependía además
de la velocidad. Se corrige con fricción anisótropa más una vía efectiva
calibrada contra la pose real de Gazebo; el error de guiñada baja del 3 % en todo
el rango de 0.1 a 1.0 rad/s.

Desarrollo completo en [cinematica.md §6](./cinematica.md).

---

## Parámetros principales

### Geometría

| Parámetro | Símbolo | Valor | Fuente |
|-----------|---------|-------|--------|
| Radio de rueda | $r$ | 0.0381 m | `model.sdf` |
| Vía efectiva | $W$ | 0.1679 m 🎯 | `model.sdf`, calibrada |
| Vía de contacto | $W_c$ | 0.17635 m | Orígenes de junta |
| Distancia entre ejes | $L$ | 0.13564 m | Orígenes de junta |
| Huella real | — | 0.217 × 0.222 m | Meshes visuales |
| Radio circunscrito | — | 0.1577 m | Meshes visuales |
| Masa total | $m$ | 5.525 kg | `model.sdf` |

### Límites

Los mismos en las tres capas (DWB, velocity smoother y plugin):

| Parámetro | Valor |
|-----------|-------|
| Velocidad lineal máxima | 0.26 m/s |
| Velocidad angular máxima | 1.0 rad/s |
| Aceleración lineal máxima | 1.0 m/s² |
| Aceleración angular máxima | 3.2 rad/s² |

---

## Estructura

```
Skid-steer 4WD
│
├─ Cinemática directa   (ω_L, ω_R) → (v, ω)
├─ Cinemática inversa   (v, ω) → (ω_L, ω_R)
│
├─ Control  gz-sim-diff-drive-system   (Ignition Fortress)
│  ├─ Entrada:  /cmd_vel   (Twist)
│  └─ Salida:   /odom      (Odometry) + TF odom → base_link vía odom_to_tf
│
├─ Percepción  YDLIDAR X3 PRO, 0.12–8 m, 7 Hz, 571 pts/vuelta
│
└─ Navegación  Nav2 Humble
   ├─ AMCL          localización sobre mapa conocido
   ├─ NavFn         plan global
   └─ DWB           controlador local
```

---

## Validación

Todo se contrasta contra la **pose real de Gazebo**, no contra la odometría:

| Medida | Resultado |
|--------|-----------|
| Error de guiñada de la odometría, 0.1–1.0 rad/s | < 3 % |
| Error de traslación de la odometría | exacto a cualquier velocidad |
| Error de pose de SLAM, recorrido de 90 m | 0.033 m de media, 0.14 m máximo |
| Puntuación del mapa contra la planta exacta | 99.8 / 100 |
| Navegación | 5/5 objetivos, 0 recuperaciones, < 0.15 m |

---

## Convenciones

1. Sistema dextrógiro: $x$ adelante, $y$ izquierda, $z$ arriba
2. Ángulos positivos en sentido antihorario
3. Velocidades expresadas en el frame del robot $\{R\}$
4. Cuatro ruedas fijas en dos pares sincronizados, sin dirección

---

**Autor**: Mario David Alvarez Vallejo
**Proyecto**: Robot Axioma — Logística Industrial Autónoma
