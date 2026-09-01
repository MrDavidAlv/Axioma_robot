# Control System of the Axioma Robot

> There is no PID anywhere in the project. The Gazebo plugin sets joint
> velocities and the physics does the rest. This document describes which layer
> enforces each limit and, above all, which of the parameters that look like
> limits **do nothing**.

## 1. Architecture

```
Nav2 behavior tree
    |   NavFn                    global plan
    |   DWB                      local trajectory
    |   velocity smoother        acceleration ramps
    v
/cmd_vel  (geometry_msgs/Twist)
    v
gz-sim-diff-drive-system         inverse kinematics
    |
    v
4 wheel joints (2 synchronised pairs)
    v
Physics (dartsim)                friction, scrub, contact
    v
/odom  +  EKF or odom_to_tf  ->  TF odom → base_footprint
    v
AMCL  ->  TF map → odom
```

---

## 2. The control plugin

### 2.1 Which one it is

**`gz-sim-diff-drive-system`**, from Ignition Gazebo Fortress.

>  It is **not** `libgazebo_ros_diff_drive.so`. That is the Gazebo Classic
> plugin and it has a different set of tags. Confusing the two is the cause of
> section 2.3.

### 2.2 Actual configuration

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

### 2.3 Tags the plugin ignores

The model used to carry these two, inherited from Gazebo Classic:

```xml
<max_wheel_torque>20</max_wheel_torque>
<max_wheel_acceleration>1</max_wheel_acceleration>
```

`gz-sim-diff-drive-system` **does not read them**, and it does not warn. The
consequence: the robot had no acceleration limit at all while DWB planned on
the assumption that it did. You can check against the installed library:

```bash
strings /usr/lib/x86_64-linux-gnu/libignition-gazebo6-diff-drive-system.so \
  | grep -xE 'max_(wheel_)?(torque|acceleration|linear_acceleration)'
```

Only the `linear`/`angular` variants show up. That is why section 2.2 uses
`max_linear_acceleration` and `max_angular_acceleration`, with the same values
as Nav2.

### 2.4 What it does each cycle

1. Reads `/cmd_vel`
2. Saturates $(v, \omega)$ against the velocity and acceleration limits
3. Applies the inverse kinematics (see `kinematics.md`, §5)
4. Writes the target velocity to the 4 joints
5. Integrates the **actual joint positions** for the odometry
6. Publishes `/odom` at 50 Hz

Step 5 matters: the odometry comes from what the wheels actually turned, not
from the command. If a wheel slips, the odometry believes it, exactly as an
encoder does on the physical robot.

---

## 3. Where each limit is enforced

| Quantity | DWB | Velocity smoother | Plugin | Physics |
|----------|-----|-------------------|--------|--------|
| $v$ | 0.26 m/s | 0.26 m/s | 0.26 m/s | friction |
| $\omega$ | 1.0 rad/s | 1.0 rad/s | 1.0 rad/s | lateral scrub |
| $\dot v$ | 1.0 m/s² | 1.0 m/s² | 1.0 m/s² | $\mu g$ |
| $\dot\omega$ | 3.2 rad/s² | 3.2 rad/s² | 3.2 rad/s² | — |

All three software layers now carry the same value. When they disagreed, DWB
chose trajectories the robot could not follow and the controller ended up
triggering recovery behaviours.

**Friction limit** (a physical upper bound, far above the ones above):

$$
a_{\mu} = \mu_2 \, g = 1.0 \times 9.81 = 9.81 \text{ m/s}^2
$$

---

## 4. Local controller: DWB

**Plugin**: `dwb_core::DWBLocalPlanner`, at `controller_frequency: 20.0` Hz.

### 4.1 Dynamic window

```yaml
max_vel_x: 0.26          min_vel_x: 0.0
max_vel_theta: 1.0       max_speed_xy: 0.26
max_vel_y: 0.0           vy_samples: 1      # differential drive: no y axis
vx_samples: 20           vtheta_samples: 20
sim_time: 1.7
acc_lim_x: 1.0           decel_lim_x: -1.0
acc_lim_theta: 3.2       decel_lim_theta: -3.2
```

`vy_samples: 1` because the robot has no lateral mobility; sampling more
velocities in $y$ only burns compute generating identical trajectories.

### 4.2 Cost functions

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

### 4.3 Progress checker

```yaml
required_movement_radius: 0.10   # m
movement_time_allowance:  10.0   # s
```

Nav2's default radius is 0.5 m, too much for a robot that tops out at
0.26 m/s: it would take 2 s just to cover it. With 0.10 m and 10 s, a stall
triggers recovery in a tenth of the time it did with the 30 s this project
carried at one point.

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

For each axis:

$$
v_k = \mathrm{clip}\big(v_{des},\; v_{k-1} - a_{max}\Delta t,\; v_{k-1} + a_{max}\Delta t\big),
\qquad \Delta t = 0.05 \text{ s}
$$

---

## 6. Localisation: AMCL

```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
alpha1 ... alpha5: 0.05
laser_min_range: 0.12      laser_max_range: 8.0
max_beams: 120             min_particles: 1000    max_particles: 5000
update_min_d: 0.1 m        update_min_a: 0.1 rad
set_initial_pose: true     initial_pose: (0, 0, 0)
```

The $\alpha$ values are the variances of the differential motion model:

$$
\sigma_{rot1}^2 = \alpha_1 |\Delta\theta| + \alpha_2 \|\Delta \mathbf{p}\|
$$
$$
\sigma_{trans}^2 = \alpha_3 \|\Delta \mathbf{p}\| + \alpha_4 |\Delta\theta|
$$

`laser_max_range` **must** match the sensor's range. When it said 3.5 m with an
8 m YDLIDAR X3 PRO, AMCL threw away more than half the returns.

---

## 7. Measured accuracy

All validation is done against Gazebo's ground-truth pose, published on
`/world/default/dynamic_pose/info` and bridged to ROS. Not against the
odometry: comparing the odometry with itself proves nothing.

**90 m mapping run through all four rooms:**

| | Mean | p95 | Maximum |
|---|---|---|---|
| Position error, raw odometry | 1.37 m | 3.27 m | 3.79 m |
| Position error, SLAM Toolbox | 0.033 m | 0.109 m | 0.141 m |
| Yaw error, SLAM Toolbox | 1.09° | — | 10.2° |

**Navigation over the resulting map:** 5 of 5 goals, zero recovery behaviours,
final error between 0.05 and 0.15 m.

### 7.1 On the physical robot

Odometry in simulation is exact in translation because there is no encoder
noise. On the real robot you also have to account for:

1. Wheel slip: inherent to skid-steer, and surface-dependent
2. Encoder resolution: 1000 PPR
3. Diameter spread between wheels

The effective track $W = 0.1679$ m is calibrated **for the simulated friction**.
On the physical robot the calibration in `kinematics.md` §6.3 has to be repeated
on the working surface.

---

## 8. Limitations

### 8.1 There is no velocity loop

The plugin sets joint velocities directly. On the real robot you need:

- One PID per motor
- Velocity measurement from the encoders
- Slip compensation, or a filter that fuses IMU and odometry

The IMU half of that already exists. `model.sdf` carries an `imu` sensor at
100 Hz on `imu_link`, sitting on the chassis over the yaw axis so that spinning
in place produces no centripetal acceleration the filter would have to subtract,
and `axioma_perception` fuses it with the wheel odometry through
`robot_localization`. That is what publishes `odom -> base_footprint` by
default, with `odom_to_tf` kept only for `use_ekf:=false`. What is still missing
is the motor-side loop: the plugin still commands velocities that the physics
grants exactly.

### 8.2 No torque limit

The plugin does not enforce torque. The only bound is friction,
$F = \mu_2 m g / 4$ per wheel. A real motor will saturate long before that.

---

## 9. References

- Plugin: `src/axioma_gazebo/models/axioma_v2/model.sdf`
- Nav2: `src/axioma_navigation/config/nav2_params.yaml`
- [Nav2 DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Nav2 AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

**Author**: Mario David Alvarez Vallejo
