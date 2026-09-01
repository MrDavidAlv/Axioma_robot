# Physical Parameters of the Axioma Robot

> Every value in this document comes from a file in the repository. The ones
> that are **calibrated** rather than measured carry a  tag and a reference to
> how they were obtained.

---

## 1. Geometry

### 1.1 Wheels

| Parameter | Symbol | Value | Source |
|-----------|---------|-------|--------|
| Radius (collision and plugin) | $r$ | 0.0381 m | `model.sdf` |
| Visual mesh radius | — | 0.0405 m | `llanta.stl` |
| Collision width | $w$ | 0.030 m | `model.sdf` |
| Collision offset along local $y$ | — | +0.020 m | `model.sdf` |

> The visual mesh is 2.4 mm larger than the collision cylinder. It only affects
> appearance: the physics and the odometry use 0.0381 m.

### 1.2 Chassis and track

| Parameter | Symbol | Value | Source |
|-----------|---------|-------|--------|
| Joint track | $W_j$ | 0.13635 m | Joint origins |
| Contact track | $W_c$ | 0.17635 m | $W_j + 2 \times 0.02$ |
| **Effective track (plugin)** | $W$ | **0.1679 m** | `model.sdf`, see `kinematics.md` §6 |
| Wheelbase | $L$ | 0.13564 m | Joint origins |
| Real length | — | 0.2166 m | Visual meshes |
| Real width | — | 0.2220 m | Visual meshes |
| Real height | — | 0.1548 m | Visual meshes |
| Circumscribed radius from `base_link` | — | 0.1577 m | Visual meshes |

`base_link` sits 2.58 mm below the ground plane: the wheel centre is at
$z = 0.04068$ and the radius is 0.0381, so $0.0381 - 0.04068 = -0.00258$.

### 1.3 Wheel positions

| Wheel | $(x, y, z)$ [m] | Joint | rpy |
|-------|-----------------|-------|-----|
| W1 (front left) | (0.06442, 0.07385, 0.04068) | `base_to_wheel1` | (π, 0, π) |
| W2 (rear left) | (-0.071224, 0.07385, 0.04068) | `base_to_wheel2` | (π, 0, π) |
| W3 (rear right) | (-0.071224, -0.0625, 0.04068) | `base_to_wheel3` | (0, 0, π) |
| W4 (front right) | (0.064423, -0.0625, 0.04068) | `base_to_wheel4` | (0, 0, π) |

The rpy values make each joint's declared axis (`0 1 0` on the left, `0 -1 0`
on the right) point along the same $+y$ of `base_link`, so a positive rotation
means forward travel in all four cases.

---

## 2. Inertial properties

### 2.1 Mass

| Component | Count | Unit mass | Total |
|------------|----------|---------------|-------|
| `base_link` (chassis) | 1 | 5.000 kg | 5.000 kg |
| Wheels | 4 | 0.100 kg | 0.400 kg |
| LiDAR (`base_scan`) | 1 | 0.125 kg | 0.125 kg |
| **TOTAL** | | | **5.525 kg** |

### 2.2 Inertia tensors

$$
\mathbf{I}_{base} = \mathrm{diag}(0.1135,\ 0.4260,\ 0.5205) \text{ kg·m}^2
$$
$$
\mathbf{I}_{wheel} = \mathrm{diag}(0.0029,\ 0.0029,\ 0.0056) \text{ kg·m}^2
$$
$$
\mathbf{I}_{lidar} = \mathrm{diag}(0.0010,\ 0.0010,\ 0.0010) \text{ kg·m}^2
$$

---

## 3. Dynamic parameters

### 3.1 Wheel-ground friction

```xml
<friction>
  <ode>
    <fdir1>0 0 1</fdir1>   <!-- cylinder axis = wheel axle -->
    <mu>0.05</mu>          <!-- lateral (scrub) -->
    <mu2>1.0</mu2>         <!-- rolling (traction) -->
    <slip1>0</slip1>
    <slip2>0</slip2>
  </ode>
</friction>
```

| Parameter | Value | What it is |
|-----------|-------|--------|
| `fdir1` | (0, 0, 1) | First friction direction, in the collision frame |
| $\mu$ | 0.05 | Coefficient **along `fdir1`**: lateral |
| $\mu_2$ | 1.0 | Perpendicular coefficient: rolling |

Without `fdir1` the physics engine picks an arbitrary direction and $\mu$/$\mu_2$
stop meaning what you expect. See `kinematics.md` §6.3.

**Maximum acceleration from friction**:
$$
a_{\mu} = \mu_2 \, g = 9.81 \text{ m/s}^2
$$

### 3.2 Contact (ODE)

| Parameter | Value |
|-----------|-------|
| `soft_cfm` | 0 |
| `soft_erp` | 0.2 |
| `kp` | $10^{13}$ |
| `kd` | 1.0 |
| `max_vel` | 0.01 m/s |
| `min_depth` | 0.01 m |

### 3.3 Physics engine

**dartsim** (the default in Ignition Fortress). The world's
`<physics type="ode">` tag is inherited and gz-sim ignores it; the `<ode>`
friction parameters are translated.

---

## 4. Operational limits

### 4.1 Kinematic

| Parameter | Value | Enforced in |
|-----------|-------|-------------|
| $v_{max}$ | 0.26 m/s | DWB, smoother, plugin |
| $\omega_{max}$ | 1.0 rad/s | DWB, smoother, plugin |
| $a_{max}$ | 1.0 m/s² | DWB, smoother, plugin |
| $\alpha_{max}$ | 3.2 rad/s² | DWB, smoother, plugin |

### 4.2 Tags with no effect

`max_wheel_torque` and `max_wheel_acceleration` belong to Gazebo Classic.
`gz-sim-diff-drive-system` does not read them. They were replaced by
`max_linear_acceleration` / `max_angular_acceleration`, which it does read.
Details in `control.md` §2.3.

### 4.3 Joints

```xml
<limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
```

Free rotation, no declared torque limit.

---

## 5. LiDAR sensor

### 5.1 YDLIDAR X3 PRO

| Parameter | Value |
|-----------|-------|
| Position | $(0, 0, 0.15)$ m on `base_link` |
| Frame | `base_scan` |
| Topic | `/scan` |
| Range | 0.12 – 8.0 m |
| Field of view | 360°, from $-\pi$ to $+\pi - \Delta$ |
| Samples per revolution | 400 |
| Angular resolution | 0.900° |
| Rotation frequency | 10 Hz (adjustable 5–10 Hz) |
| Sample rate | 4 kHz |
| Noise | Gaussian, $\sigma = 0.015$ m |
| Ambient light rejection | 40 klux |

The resolution is **not a fixed property of the sensor**: it is single-channel,
so

$$
\text{samples per revolution} = \frac{\text{sample rate}}{\text{rotation frequency}}
= \frac{4000}{10} = 400
$$

| Frequency | Samples/revolution | Resolution |
|------------|-----------------|------------|
| 5 Hz | 800 | 0.450° |
| 7 Hz | 571 | 0.630° |
| **10 Hz** | **400** | **0.900°** |

If `<update_rate>` changes, `<samples>` has to change in the same proportion.
The physical robot runs at the same frequency:
`axioma_bringup/config/ydlidar_x3_pro.yaml` asks the head for 10 Hz.

### 5.2 Transform

$$
\mathbf{T}^{base}_{scan} =
\begin{bmatrix} 1&0&0&0 \\ 0&1&0&0 \\ 0&0&1&0.15 \\ 0&0&0&1 \end{bmatrix}
$$

No rotation: the scan already comes aligned with `base_link`.

---

## 6. Navigation

### 6.1 Costmaps

| | Local | Global |
|---|---|---|
| Update frequency | 5.0 Hz | 1.0 Hz |
| Size | 6 × 6 m (rolling) | map size |
| Resolution | 0.05 m | 0.05 m |
| `robot_radius` | 0.16 m | 0.16 m |
| `inflation_radius` | 0.25 m | 0.30 m |
| `cost_scaling_factor` | 3.0 | 3.0 |
| Layers | obstacle, inflation | static, obstacle, inflation |
| Raytrace range | 6.0 m | 8.0 m |

The local one raytraces up to 6 m because its window is 6 × 6: beyond that
there is no cell to update.

### 6.2 Tolerances

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

The new-node thresholds (0.2 m / 0.2 rad) have to stay **inside** the scan
matcher's search window (±0.25 m from `correlation_search_space_dimension`,
±20° from `coarse_search_angle_offset`). With the original 0.5/0.5 the matcher
could not close the correspondence and the graph diverged.

### 7.1 Map

| Parameter | Value |
|-----------|-------|
| Origin | (-7.01, -2.99) m |
| Resolution | 0.05 m/pixel |
| Thresholds | occupied 0.65, free 0.25 |

Produced by SLAM Toolbox driving through `office.world`, not derived
analytically. `maps/ground_truth.pgm` *is* the exact floor plan, and it is what
scores the SLAM map through `axioma_slam/scripts/score_map.py`.

---

## 8. Derived calculations

### 8.1 Wheel speeds

$$
\omega_{straight} = \frac{v_{max}}{r} = \frac{0.26}{0.0381} = 6.82 \text{ rad/s} = 65.2 \text{ RPM}
$$
$$
\omega_{turn} = \frac{\omega_{max} W}{2r} = \frac{1.0 \times 0.1679}{2 \times 0.0381} = 2.20 \text{ rad/s} = 21.0 \text{ RPM}
$$
$$
\omega_{comb} = \frac{v_{max} + \omega_{max} W/2}{r} = 9.03 \text{ rad/s}
$$

### 8.2 Braking and turning

$$
d_{braking} = \frac{v_{max}^2}{2 |a_{min}|} = \frac{0.26^2}{2 \times 1.0} = 0.034 \text{ m}
$$
$$
R_{min} = 0 \text{ m}, \qquad
R\big|_{max} = \frac{v_{max}}{\omega_{max}} = 0.26 \text{ m}
$$

---

## 9. Summary

```python
GEOMETRY = {
    'wheel_radius':      0.0381,    # m   model.sdf
    'wheel_separation':  0.1679,    # m   model.sdf, calibrated
    'contact_track':     0.17635,   # m   derived from the joints
    'wheel_base':        0.13564,   # m   derived from the joints
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
    'lidar':          10.0,   # Hz   model.sdf
    'imu':           100.0,   # Hz   model.sdf
    'zed2i':          15.0,   # Hz   model.sdf
    'local_costmap':   5.0,   # Hz   nav2_params.yaml
}
```

---

## 10. Inconsistencies

### 10.1 Wheel track — resolved

The earlier version of this document already pointed out that
`wheel_separation` (0.1725 m) did not match the joint separation (0.13635 m)
and warned that it "may cause odometry drift during rotations".

It was right. Measured: the robot turned **65 %** of what was commanded. It was
fixed with anisotropic friction plus a calibrated effective track; yaw error
came down below 3 %. Procedure in `kinematics.md` §6.

### 10.2 Chassis mass — resolved

The URDF declared 1.0 kg and a tensor different from the SDF's. Both now carry
5.0 kg and $\mathrm{diag}(0.1135,\ 0.426,\ 0.5205)$.

### 10.3 Inertia on the root link — resolved

`robot_state_publisher` warned on every startup:

> The root link base_link has an inertia specified in the URDF, but KDL does not
> support a root link with an inertia.

`base_footprint` was added as a root without `<inertial>`, with a fixed joint to
`base_link` offset $-0.00258$ m in $z$. The warning is gone and the TF tree ends
up in the conventional shape. See `kinematics.md` §2.5.

### 10.4 `imu_link` with no sensor — resolved

The URDF used to declare `imu_link` at $(0.1, 0.1, 0.12)$, outside the chassis
envelope, with no IMU sensor anywhere in the SDF: an orphan TF frame. The link
now sits on the chassis and `model.sdf` carries a real `imu` sensor at 100 Hz,
fused with the wheel odometry by `robot_localization` in `axioma_perception` —
which is what makes roll and pitch measured rather than assumed. Both worlds
load `gz-sim-imu-system`, without which an IMU publishes nothing and logs no
warning.

### 10.5 Wheel mesh radius — informational

Visual 0.0405 m against collision 0.0381 m. Cosmetic only.

---

## 11. Cross-references

- **Kinematics**: [kinematics.md](./kinematics.md)
- **Control**: [control.md](./control.md)
- **Diagram**: [render_diagram.py](./render_diagram.py) generates
  `images/mathematical-model.png` by reading these same files
- **Source files**:
  - `src/axioma_gazebo/models/axioma_v2/model.sdf`
  - `src/axioma_description/urdf/axioma.urdf.xacro`
  - `src/axioma_navigation/config/nav2_params.yaml`
  - `src/axioma_slam/config/slam_params.yaml`

---

**Author**: Mario David Alvarez Vallejo
