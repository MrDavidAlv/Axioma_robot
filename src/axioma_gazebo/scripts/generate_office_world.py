"""Fuente unica de la planta de oficina simulada.

Genera dos artefactos a partir de la MISMA geometria, de modo que no puedan
desincronizarse:

  * ``axioma_gazebo/worlds/office.world``      - mundo SDF para Gz Sim
  * ``axioma_navigation/maps/mapa.{pgm,yaml}`` - mapa de ocupacion para Nav2

Planta: recepcion diafana al sur, y al norte oficina 1, oficina 2 y sala de
juntas, separadas por un tabique con tres puertas de 1.2 m.

Uso:
    python3 generate_office_world.py            # regenera mundo y mapa
    python3 generate_office_world.py <ruta>     # solo el mundo, en <ruta>
"""

import os

T = 0.15   # grosor de muro
H = 1.5    # altura de muro

WALL_MAT = (0.85, 0.85, 0.82)
GLASS_MAT = (0.55, 0.70, 0.78)
FURN_MAT = (0.45, 0.32, 0.22)
SOFT_MAT = (0.25, 0.35, 0.50)

walls = []   # (name, cx, cy, sx, sy, sz, mat)


def wall(name, cx, cy, sx, sy, sz=H, mat=WALL_MAT):
    walls.append((name, cx, cy, sx, sy, sz, mat))


# ---------------- Muros exteriores (x: -7..7, y: -3..7) ----------------
wall('wall_south', 0.0, -3.0, 14.0 + T, T)
wall('wall_north', 0.0, 7.0, 14.0 + T, T)
wall('wall_west', -7.0, 2.0, T, 10.0)
wall('wall_east', 7.0, 2.0, T, 10.0)

# ------- Tabique recepcion / salas en y = 2.0, con 3 puertas de 1.2 m -------
# 1.4 m: con robot_radius 0.15 e inflation_radius 0.30 quedan 0.8 m de banda
# de coste cero en el vano, holgura comoda para el planificador local.
DOOR = 1.4
doors = [-4.75, -0.25, 4.5]          # centro de cada puerta
edges = [-7.0]
for d in doors:
    edges += [d - DOOR / 2, d + DOOR / 2]
edges += [7.0]
for i in range(0, len(edges), 2):
    a, b = edges[i], edges[i + 1]
    if b - a > 1e-6:
        wall(f'wall_partition_{i//2}', (a + b) / 2, 2.0, b - a, T)

# ---------------- Tabiques entre salas (y: 2..7) ----------------
wall('wall_div_office', -2.5, 4.5, T, 5.0)
wall('wall_div_meeting', 2.0, 4.5, T, 5.0)

# ---------------- Mobiliario: recepcion ----------------
wall('reception_desk', 0.0, -1.4, 3.0, 0.8, 1.05, FURN_MAT)
wall('sofa_left', -5.0, -1.8, 1.8, 0.7, 0.8, SOFT_MAT)
wall('sofa_right', 5.0, -1.8, 1.8, 0.7, 0.8, SOFT_MAT)

# ---------------- Mobiliario: oficina 1 ----------------
wall('office1_desk', -5.5, 5.6, 1.8, 0.8, 0.75, FURN_MAT)
wall('office1_cabinet', -6.6, 3.2, 0.5, 1.6, 1.8, FURN_MAT)
wall('office1_chair', -5.5, 4.7, 0.45, 0.45, 0.5, SOFT_MAT)

# ---------------- Mobiliario: oficina 2 ----------------
wall('office2_desk', -0.9, 5.6, 1.8, 0.8, 0.75, FURN_MAT)
wall('office2_cabinet', 1.6, 3.4, 0.5, 1.6, 1.8, FURN_MAT)
wall('office2_chair', -0.9, 4.7, 0.45, 0.45, 0.5, SOFT_MAT)

# ---------------- Mobiliario: sala de juntas ----------------
wall('meeting_table', 4.5, 5.0, 3.0, 1.4, 0.75, FURN_MAT)
for i, (cx, cy) in enumerate([(3.3, 3.9), (5.7, 3.9), (3.3, 6.1), (5.7, 6.1)]):
    wall(f'meeting_chair_{i}', cx, cy, 0.45, 0.45, 0.5, SOFT_MAT)
wall('meeting_whiteboard', 6.8, 5.0, 0.1, 2.0, 1.2, GLASS_MAT)

# ---------------- Pilares (cilindros) ----------------
# Columnas repartidas cada ~3 m: la recepcion mide 14 m y el LiDAR solo llega a
# 3.5 m; sin ellas el scan matching queda sin referencia en el eje X (problema de
# apertura) y el grafo de SLAM deriva a lo largo del pasillo.
pillars = [('pillar_1', -5.0, 0.6, 0.25, H),
           ('pillar_2', -2.0, 0.6, 0.25, H),
           ('pillar_3', 1.0, 0.6, 0.25, H),
           ('pillar_4', 4.0, 0.6, 0.25, H),
           ('planter_corner', -6.2, 1.2, 0.30, 0.9)]


def box_model(name, cx, cy, sx, sy, sz, mat):
    r, g, b = mat
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx} {cy} {sz/2:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{r*0.5:.3f} {g*0.5:.3f} {b*0.5:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def cyl_model(name, cx, cy, radius, height):
    r, g, b = WALL_MAT
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx} {cy} {height/2:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        </collision>
        <visual name='visual'>
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
          <material>
            <ambient>{r*0.5:.3f} {g*0.5:.3f} {b*0.5:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


body = ''.join(box_model(*w) for w in walls)
body += ''.join(cyl_model(*p) for p in pillars)

world = f"""<sdf version='1.9'>
  <world name='default'>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <gravity>0 0 -9.8</gravity>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light type='directional' name='sun'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.40 0.40 0.42 1</ambient>
            <diffuse>0.55 0.55 0.57 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
{body}

  </world>
</sdf>
"""

# ---------------------------------------------------------------- mapa 2D ----
# El LiDAR va a z = 0.15 m, asi que solo cuenta como obstaculo lo que lo supere.
LIDAR_Z = 0.15
RES = 0.05            # m/celda, igual que resolution en nav2_params
MARGIN = 1.0          # m de relleno alrededor del edificio
SPAWN = (0.0, 0.0)    # el robot nace aqui; define el origen del frame map

FREE, OCC, UNKNOWN = 254, 0, 205


def build_map():
    import collections
    solids = [(cx, cy, sx, sy) for (_, cx, cy, sx, sy, sz, _) in walls if sz > LIDAR_Z]
    disks = [(cx, cy, r) for (_, cx, cy, r, hh) in pillars if hh > LIDAR_Z]

    xs = [cx - sx / 2 for cx, cy, sx, sy in solids] + [cx - r for cx, cy, r in disks]
    xs += [cx + sx / 2 for cx, cy, sx, sy in solids] + [cx + r for cx, cy, r in disks]
    ys = [cy - sy / 2 for cx, cy, sx, sy in solids] + [cy - r for cx, cy, r in disks]
    ys += [cy + sy / 2 for cx, cy, sx, sy in solids] + [cy + r for cx, cy, r in disks]

    ox, oy = min(xs) - MARGIN, min(ys) - MARGIN
    w = int(round((max(xs) + MARGIN - ox) / RES))
    h = int(round((max(ys) + MARGIN - oy) / RES))
    grid = [[UNKNOWN] * w for _ in range(h)]

    def cell(x, y):
        return int((x - ox) / RES), int((y - oy) / RES)

    for cx, cy, sx, sy in solids:
        c0, r0 = cell(cx - sx / 2, cy - sy / 2)
        c1, r1 = cell(cx + sx / 2, cy + sy / 2)
        for r in range(max(0, r0), min(h, r1 + 1)):
            for c in range(max(0, c0), min(w, c1 + 1)):
                grid[r][c] = OCC
    for cx, cy, rad in disks:
        c0, r0 = cell(cx - rad, cy - rad)
        c1, r1 = cell(cx + rad, cy + rad)
        for r in range(max(0, r0), min(h, r1 + 1)):
            for c in range(max(0, c0), min(w, c1 + 1)):
                x, y = ox + (c + 0.5) * RES, oy + (r + 0.5) * RES
                if (x - cx) ** 2 + (y - cy) ** 2 <= rad ** 2:
                    grid[r][c] = OCC

    # Espacio libre = lo alcanzable desde el spawn sin cruzar un obstaculo.
    # Lo de fuera del edificio queda como desconocido, igual que en un mapa real.
    sc, sr = cell(*SPAWN)
    assert grid[sr][sc] != OCC, 'el spawn cae dentro de un obstaculo'
    q = collections.deque([(sr, sc)])
    grid[sr][sc] = FREE
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and grid[rr][cc] == UNKNOWN:
                grid[rr][cc] = FREE
                q.append((rr, cc))
    return grid, w, h, ox, oy


def write_map(pgm_path, yaml_path):
    grid, w, h, ox, oy = build_map()
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        for r in range(h - 1, -1, -1):       # PGM: primera fila = y maxima
            f.write(bytes(grid[r]))
    with open(yaml_path, 'w') as f:
        f.write(f"""image: {os.path.basename(pgm_path)}
mode: trinary
resolution: {RES}
origin: [{ox:.3f}, {oy:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")
    occ = sum(row.count(OCC) for row in grid)
    free = sum(row.count(FREE) for row in grid)
    print(f'mapa {w}x{h} ({w*RES:.2f} x {h*RES:.2f} m) origen=({ox:.2f},{oy:.2f}) '
          f'ocupado={occ} libre={free} -> {pgm_path}')


if __name__ == '__main__':
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.dirname(os.path.dirname(here))

    if len(sys.argv) > 1:
        world_path = sys.argv[1]
        map_dir = None
    else:
        world_path = os.path.join(src, 'axioma_gazebo', 'worlds', 'office.world')
        map_dir = os.path.join(src, 'axioma_navigation', 'maps')

    open(world_path, 'w').write(world)
    print(f'mundo {len(walls)} cajas + {len(pillars)} cilindros -> {world_path}')

    if map_dir:
        os.makedirs(map_dir, exist_ok=True)
        write_map(os.path.join(map_dir, 'mapa.pgm'),
                  os.path.join(map_dir, 'mapa.yaml'))
