# 📡 Validación de SLAM

El README muestra el resultado principal (puntuación del mapa contra el plano
exacto). Aquí está el resto de la evidencia: cómo se mide, y por qué la
odometría cruda no basta.

---

## Una sola fuente de verdad para el mundo y su plano

`src/axioma_gazebo/scripts/generate_office_world.py` emite **a la vez** el
mundo de Gazebo y una rejilla de ocupación exacta de ese mismo mundo, a partir
de la misma lista de geometría, para que los dos no puedan desalinearse:

```bash
python3 src/axioma_gazebo/scripts/generate_office_world.py
# -> src/axioma_gazebo/worlds/office.world
# -> src/axioma_navigation/maps/ground_truth.{pgm,yaml}
```

Solo se rasteriza como obstáculo la geometría más alta que el plano del LiDAR
(0.15 m), y el espacio libre se rellena por inundación desde el punto de
spawn, así que todo lo que queda fuera del edificio permanece desconocido: la
misma estructura que produce un mapeo real.

`ground_truth` **no** es el mapa con el que navega Nav2. Ese
(`maps/mapa.yaml`) sale de conducir el robot con SLAM Toolbox corriendo,
igual que en el robot real. El plano exacto existe para poder **medir** la
calidad del mapa de SLAM en vez de solo mirarlo.

## Cómo se puntúa

`src/axioma_slam/scripts/score_map.py` compara el mapa de SLAM contra ese
plano exacto y reporta dos números: qué tan lejos está cada celda ocupada del
mapa de SLAM del obstáculo real más cercano (cuánto del mapa está inventado),
y cuántos obstáculos reales dentro del territorio explorado el mapa realmente
marca.

```bash
python3 src/axioma_slam/scripts/score_map.py --saved out.png "titulo del run"
```

## Deriva de la odometría vs. SLAM

La odometría cruda (rueda) deriva a **3.79 m** a lo largo del recorrido,
mientras que SLAM Toolbox se mantiene dentro de **0.14 m** de la pose real de
Gazebo todo el tiempo, sin saltos.

<div align="center">
<img src="../images/slam-telemetry.png" width="960"/>
</div>

## Verificación de localización

Retornos de láser en vivo (verde) proyectados sobre el mapa de SLAM y sobre el
costmap global inflado, con la pose de AMCL en rojo. Caen sobre las paredes,
el mostrador de recepción y las columnas, confirmando que el mundo, el mapa y
la estimación de pose concuerdan entre sí.

<div align="center">
<img src="../images/costmap-validation.png" width="960"/>
</div>

---

Vuelve al [README](../README.md).
