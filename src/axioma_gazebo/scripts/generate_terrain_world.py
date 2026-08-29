"""Single source of truth for the simulated warehouse-and-yard terrain.

Emits two artefacts from the SAME geometry list, so they cannot drift apart:

  * ``axioma_gazebo/worlds/terrain.world``                     - SDF for Gz Sim
  * ``axioma_navigation/maps/terrain_ground_truth.{pgm,yaml}`` - exact plan

Same convention as ``generate_office_world.py``: primitives only, nothing
downloaded from Fuel, and the occupancy grid derived from the same list of
boxes the world is built from.

---------------------------------------------------------------------------
What this world is for
---------------------------------------------------------------------------
The office world is flat, and on flat ground wheel odometry is right. This one
exists to break two assumptions Axioma's stack makes, one per half:

  * **The ramps** break "the world is a plane". The gz DiffDrive plugin
    integrates wheel rotation as if it were horizontal travel, because a wheel
    encoder cannot tell forward from up. Driving these ramps is what makes the
    IMU-fused EKF in ``axioma_perception`` measurably better than the raw
    plugin output rather than merely differently configured.

  * **The yard** breaks "friction is a constant". The three surface patches
    carry genuinely different ``<mu>``/``<mu2>``, not just different colours.
    The effective skid-steer track ``W = 0.1679 m`` documented in
    ``documentacion/modelo-matematico/control.md`` was calibrated against the
    office world's single friction value, and that document says so. This is
    the world where that caveat becomes testable.

Layout, 18 x 10 m, driven as a loop rather than a dead end:

    warehouse (z=0, cement)  --north ramp up-->  yard (z=0.28)
              ^                                        |
              +---------- south ramp down -------------+

No stairs anywhere, by design: the wheels are 3.81 cm in radius on a rigid
chassis with no suspension, so a real stair riser does not get climbed, it gets
hit. The ramps are 8.85 deg, which that geometry can actually take.

Usage:
    python3 generate_terrain_world.py            # world + ground truth
    python3 generate_terrain_world.py <path>     # world only, written to <path>
"""

import math
import os

# --------------------------------------------------------------- geometry ---
T = 0.15          # wall thickness
H = 1.5           # perimeter wall height

RISE = 0.28       # yard platform height above the warehouse floor
RUN = 1.8         # ramp horizontal length
RAMP_W = 1.6      # ramp width
RAMP_X0 = -1.0    # ramp foot (warehouse side)
RAMP_X1 = RAMP_X0 + RUN
RAMP_T = 0.02     # ramp plate thickness; see ramp() for why it is this thin

# The plate rests ON the floor, so its top face starts one thickness up and the
# slope only has to cover the rest of the rise. Thickness measured vertically is
# RAMP_T/cos(theta), and theta depends on it, so solve the pair by iterating —
# it converges in two passes at this scale.
THETA = math.atan2(RISE - RAMP_T, RUN)
for _ in range(4):
    RAMP_LIP = RAMP_T / math.cos(THETA)
    THETA = math.atan2(RISE - RAMP_LIP, RUN)
RAMP_L = math.hypot(RUN, RISE - RAMP_LIP)

RAMP_Y = 2.5      # north ramp centreline; the south one mirrors it

# Friction. mu is along the wheel's rolling direction, mu2 across it, and for a
# skid-steer the second one is not decoration: turning in place *is* a sideways
# scrub of all four tyres. Values are the usual dry-contact figures for each
# surface, kept equal in both directions because none of these surfaces is
# anisotropic.
MU_CEMENT = 1.00
MU_ASPHALT = 0.90
MU_DIRT = 0.55
MU_SAND = 0.30

CEMENT = (0.62, 0.62, 0.60)
ASPHALT = (0.24, 0.24, 0.26)
DIRT = (0.45, 0.33, 0.20)
SAND = (0.78, 0.68, 0.45)
WALL_MAT = (0.85, 0.85, 0.82)
STEEL = (0.35, 0.45, 0.55)
CRATE = (0.55, 0.40, 0.24)
DRUM = (0.70, 0.45, 0.15)

# (name, cx, cy, cz, sx, sy, sz, mat, mu, pitch, kind)
#   cz     centre height, so a slab resting on the floor has cz = sz/2
#   pitch  rotation about y, radians, negative tilts the +x end upwards
#   kind   'floor' is drivable and never enters the occupancy grid;
#          'obstacle' is what the LiDAR sees
boxes = []
cylinders = []    # (name, cx, cy, base_z, radius, height, mat, kind)


def box(name, cx, cy, cz, sx, sy, sz, mat=WALL_MAT, mu=MU_CEMENT,
        pitch=0.0, kind='obstacle'):
    boxes.append((name, cx, cy, cz, sx, sy, sz, mat, mu, pitch, kind))


def cyl(name, cx, cy, base_z, radius, height, mat=STEEL, kind='obstacle'):
    cylinders.append((name, cx, cy, base_z, radius, height, mat, kind))


def wall(name, cx, cy, sx, sy, sz=H, mat=WALL_MAT):
    """Perimeter/partition wall standing on the warehouse floor."""
    box(name, cx, cy, sz / 2, sx, sy, sz, mat)


# ---------------------------------------------------------------- perimeter --
wall('wall_west', -9.0, 0.0, T, 10.0 + T)
wall('wall_east', 9.0, 0.0, T, 10.0 + T)
wall('wall_south', 0.0, -5.0, 18.0 + T, T)
wall('wall_north', 0.0, 5.0, 18.0 + T, T)


# -------------------------------------------------------------------- ramps --
def ramp(name, cy):
    """A thin tilted plate, resting on the floor at its low end.

    Solved for the BOTTOM edge of the low end, which is placed exactly on z=0.
    The obvious alternative — a thick slab whose top face meets z=0 flush, with
    the rest of it buried under the ground plane — was built first and is
    wrong. It leaves a strip at the foot of the ramp where a wheel is in
    contact with the ground plane and with the slab's top face just below it at
    the same time; the solver resolves that ambiguity by ejecting the robot,
    which looks exactly like the robot sinking into the floor and then being
    flung up the ramp.

    So the plate rests on top instead, and the price is a 2 cm lip at the foot.
    That is a real bump on 3.81 cm wheels with no suspension, but it is a bump
    the robot demonstrably drives over: this is the same geometry that produced
    the ramp measurements in the README, 1 mm of error against ground truth
    over a 10 deg climb.
    """
    # Two conditions fix the plate: its top face ends flush with the platform
    # at (RAMP_X1, RISE), and its low bottom corner rests on the floor at z=0.
    # Solve for the top-face midpoint, then step half a thickness down the
    # plate's own normal to get the centre the SDF pose wants.
    mid_x = (RAMP_X0 + RAMP_X1) / 2
    mid_z = (RAMP_LIP + RISE) / 2
    cx = mid_x + (RAMP_T / 2) * math.sin(THETA)
    cz = mid_z - (RAMP_T / 2) * math.cos(THETA)
    box(name, cx, cy, cz, RAMP_L, RAMP_W, RAMP_T,
        CEMENT, MU_CEMENT, -THETA, kind='floor')


ramp('ramp_north', RAMP_Y)
ramp('ramp_south', -RAMP_Y)


# ------------------------------------------------------- yard platform -------
# Three strips, each its own body so each can carry its own friction. They butt
# up against each other in x; the seams are at x = 3.5 and x = 6.2.
def strip(name, x0, x1, mat, mu):
    box(name, (x0 + x1) / 2, 0.0, RISE / 2, x1 - x0, 10.0, RISE,
        mat, mu, kind='floor')


strip('yard_asphalt', RAMP_X1, 3.5, ASPHALT, MU_ASPHALT)
strip('yard_dirt', 3.5, 6.2, DIRT, MU_DIRT)
strip('yard_sand', 6.2, 9.0, SAND, MU_SAND)

# Kerb along the platform's west face, everywhere the ramps do not open onto
# it. Without this the only thing between a robot driving west across the yard
# and a 28 cm drop is good intentions. 0.5 m tall so it is well above the
# LiDAR plane, which sits 0.15 m over whatever surface the robot is on.
KERB_T = 0.15
for i, (y0, y1) in enumerate([(-5.0, -RAMP_Y - RAMP_W / 2),
                              (-RAMP_Y + RAMP_W / 2, RAMP_Y - RAMP_W / 2),
                              (RAMP_Y + RAMP_W / 2, 5.0)]):
    box(f'yard_kerb_{i}', RAMP_X1 + KERB_T / 2, (y0 + y1) / 2,
        RISE + 0.25, KERB_T, y1 - y0, 0.5, CEMENT)

# ---------------------------------------------------------------------------
# Obstacles.
#
# Same design rule as the office world: NO two places may look alike to a 3.5 m
# LiDAR. Repeated, symmetric racking is exactly what makes a scan matcher find
# false correspondences and duplicate walls in the map, and a warehouse is the
# most tempting place in the world to lay out identical rows. So the racks have
# different lengths and irregular spacing, and the two halves are furnished
# from different vocabularies: rectilinear steel indoors, round drums outside.
# ---------------------------------------------------------------------------

# ------- Warehouse racking -------------------------------------------------
# Positions are constrained by _check_layout() below: nothing may sit in either
# ramp's approach corridor, and nothing may overlap anything else. Move a piece
# into a corridor and the generator refuses to emit a world rather than letting
# the robot discover it at 0.22 m/s.
box('rack_a', -6.8, 4.3, 0.90, 4.0, 0.8, 1.8, STEEL)
box('rack_b', -5.4, 0.6, 0.75, 5.6, 0.8, 1.5, STEEL)
box('rack_c', -7.3, -4.2, 1.05, 3.0, 0.8, 2.1, STEEL)
box('rack_d', -3.1, -4.3, 0.60, 2.2, 0.9, 1.2, STEEL)

# Pallets and a stack of goods: low, but well above the 0.15 m LiDAR plane.
box('pallet_1', -8.1, -0.8, 0.20, 1.2, 1.0, 0.40, CRATE)
box('pallet_2', -2.0, -0.6, 0.30, 1.0, 1.2, 0.60, CRATE)
box('crate_stack', -4.2, -0.9, 0.55, 0.9, 0.9, 1.1, CRATE)

# Columns holding the roof up. Different radii, irregular spacing, for the same
# reason the office world's are: evenly spaced identical columns turn "shifted
# by one bay" into a pose the matcher cannot distinguish from "not shifted".
cyl('col_a', -6.2, -1.0, 0.0, 0.22, 2.4)
cyl('col_b', -3.9, 4.2, 0.0, 0.28, 2.4)
cyl('col_c', -1.9, 1.0, 0.0, 0.19, 2.4)

# ------- Yard. Everything here stands on the platform, so base_z = RISE ------
cyl('drum_a', 2.3, 3.4, RISE, 0.29, 0.9, DRUM)
cyl('drum_b', 2.9, 3.9, RISE, 0.29, 0.9, DRUM)
cyl('drum_c', 5.1, -3.9, RISE, 0.29, 0.9, DRUM)
box('yard_crate_a', 4.4, 2.2, RISE + 0.35, 1.1, 0.8, 0.7, CRATE)
box('yard_crate_b', 7.4, 1.1, RISE + 0.50, 0.9, 1.4, 1.0, CRATE)
box('yard_bin', 6.6, -2.6, RISE + 0.40, 1.6, 0.7, 0.8, STEEL)
box('yard_shed', 8.0, 3.6, RISE + 0.90, 1.8, 2.2, 1.8, WALL_MAT)


# ---------------------------------------------------------------------------
# Approach corridors.
#
# Each ramp is only usable if the robot can reach its foot, and "can it reach
# it" is not something to check by reading coordinates off the page. It was
# checked that way once: a pallet sat squarely across the north ramp's
# approach, the robot drove into it at 0.22 m/s, climbed it and ended up on its
# back. So the corridors are asserted here, and adding furniture that blocks
# one breaks the generator instead of the demo.
#
# The corridor is the ramp's own width, run from the west wall to the foot.
# Only obstacles matter: the ramps and the platform are 'floor'.
# ---------------------------------------------------------------------------
def _footprints():
    """Every obstacle as (name, x0, x1, y0, y1). Cylinders use their bbox."""
    out = []
    for (name, bx, by, bz, sx, sy, sz, _, _, _, kind) in boxes:
        if kind == 'obstacle':
            out.append((name, bx - sx / 2, bx + sx / 2, by - sy / 2, by + sy / 2))
    for (name, cx, cy, bz, rad, hh, _, kind) in cylinders:
        if kind == 'obstacle':
            out.append((name, cx - rad, cx + rad, cy - rad, cy + rad))
    return out


def _overlap(a, b):
    """Both args are (name, x0, x1, y0, y1)."""
    return (a[1] < b[2] and b[1] < a[2]) and (a[3] < b[4] and b[3] < a[4])


def _check_layout():
    problems = []
    fps = _footprints()

    # Ramp approaches: the robot has to be able to reach each foot.
    for cy, label in ((RAMP_Y, 'north'), (-RAMP_Y, 'south')):
        lane = ('corridor', -9.0 + T, RAMP_X0, cy - RAMP_W / 2, cy + RAMP_W / 2)
        for fp in fps:
            if _overlap(fp, lane):
                problems.append(f'{fp[0]} blocks the {label} ramp approach')

    # And no two pieces of furniture may be inside each other. Two boxes
    # sharing space look fine in a render and behave like neither of them in
    # the solver. The building shell is exempt: walls meet at the corners and
    # the kerbs run into them, both on purpose.
    shell = [fp for fp in fps if fp[0].startswith(('wall_', 'yard_kerb_'))]
    furniture = [fp for fp in fps if fp not in shell]
    for i, a in enumerate(furniture):
        for b in furniture[i + 1:]:
            if _overlap(a, b):
                problems.append(f'{a[0]} overlaps {b[0]}')
    for a in furniture:
        for b in shell:
            if _overlap(a, b):
                problems.append(f'{a[0]} overlaps {b[0]}')

    if problems:
        raise AssertionError('layout is unusable:\n  - ' + '\n  - '.join(problems))


_check_layout()


# ------------------------------------------------------------------- SDF -----
def surface(mu):
    return f"""
          <surface>
            <friction>
              <ode>
                <mu>{mu}</mu>
                <mu2>{mu}</mu2>
              </ode>
            </friction>
          </surface>"""


def material(mat):
    r, g, b = mat
    return f"""
          <material>
            <ambient>{r*0.5:.3f} {g*0.5:.3f} {b*0.5:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>"""


def box_model(name, cx, cy, cz, sx, sy, sz, mat, mu, pitch, kind):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx:.5f} {cy:.5f} {cz:.5f} 0 {pitch:.6f} 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>{surface(mu)}
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>{material(mat)}
        </visual>
      </link>
    </model>"""


def cyl_model(name, cx, cy, base_z, radius, height, mat, kind):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx:.5f} {cy:.5f} {base_z + height/2:.5f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>{surface(MU_CEMENT)}
        </collision>
        <visual name='visual'>
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>{material(mat)}
        </visual>
      </link>
    </model>"""


body = ''.join(box_model(*b) for b in boxes)
body += ''.join(cyl_model(*c) for c in cylinders)

world = f"""<sdf version='1.9'>
  <world name='default'>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <!-- IMU. Unlike the lidar and the camera, which the Sensors system above
         already covers, an <sensor type="imu"> publishes nothing at all unless
         this system is loaded in the world: the topic simply never appears and
         Gazebo logs no warning about it. This world is the one where that
         matters most — without the IMU there is no attitude, and without
         attitude the ramps below are invisible to the odometry. -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

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

    <!-- The warehouse floor. Cement, and the only surface in this world whose
         friction the office world also had, which is what makes it the
         reference the yard's three patches are compared against. -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>{surface(MU_CEMENT)}
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>{CEMENT[0]*0.5:.3f} {CEMENT[1]*0.5:.3f} {CEMENT[2]*0.5:.3f} 1</ambient>
            <diffuse>{CEMENT[0]:.3f} {CEMENT[1]:.3f} {CEMENT[2]:.3f} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
{body}

  </world>
</sdf>
"""

# ------------------------------------------------------------- 2D map --------
# The LiDAR plane sits 0.15 m above whatever surface the robot is standing on,
# so an obstacle counts if it rises more than that above its own base — which
# is why every obstacle carries its base height rather than being assumed to
# start at zero.
#
# A single 2D grid cannot express two floors, and this world has two. What it
# does express is correct for both: the walls and obstacles are the same seen
# from either level, and the ramps and platform are drivable surface, not
# obstacles, so they are excluded by their 'floor' kind rather than by height.
LIDAR_Z = 0.15
RES = 0.05
MARGIN = 1.0
SPAWN = (-5.0, 2.5)      # matches terrain_demo.launch.py: lined up with the north ramp

FREE, OCC, UNKNOWN = 254, 0, 205


def build_map():
    import collections

    solids = [(cx, cy, sx, sy) for (_, cx, cy, cz, sx, sy, sz, _, _, _, kind)
              in boxes if kind == 'obstacle' and sz > LIDAR_Z]
    disks = [(cx, cy, r) for (_, cx, cy, bz, r, hh, _, kind)
             in cylinders if kind == 'obstacle' and hh > LIDAR_Z]

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

    sc, sr = cell(*SPAWN)
    assert grid[sr][sc] != OCC, 'spawn point falls inside an obstacle'
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
        for r in range(h - 1, -1, -1):       # PGM: first row is max y
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
    print(f'map {w}x{h} ({w*RES:.2f} x {h*RES:.2f} m) origin=({ox:.2f},{oy:.2f}) '
          f'occupied={occ} free={free} -> {pgm_path}')


if __name__ == '__main__':
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.dirname(os.path.dirname(here))

    if len(sys.argv) > 1:
        world_path = sys.argv[1]
        map_dir = None
    else:
        world_path = os.path.join(src, 'axioma_gazebo', 'worlds', 'terrain.world')
        map_dir = os.path.join(src, 'axioma_navigation', 'maps')

    open(world_path, 'w').write(world)
    print(f'world: {len(boxes)} boxes + {len(cylinders)} cylinders -> {world_path}')
    print(f'ramps: {math.degrees(THETA):.2f} deg, rise {RISE} m over {RUN} m, '
          f'width {RAMP_W} m, lip at the foot {RAMP_LIP*1000:.1f} mm')
    print(f'yard friction: asphalt mu={MU_ASPHALT}, dirt mu={MU_DIRT}, '
          f'sand mu={MU_SAND} (warehouse cement mu={MU_CEMENT})')

    if map_dir:
        os.makedirs(map_dir, exist_ok=True)
        write_map(os.path.join(map_dir, 'terrain_ground_truth.pgm'),
                  os.path.join(map_dir, 'terrain_ground_truth.yaml'))
