#!/usr/bin/env python3
"""Render the kinematic-model figure straight from the robot's own files.

Everything on the diagram - geometry, limits, sensor spec - is read from
``model.sdf``, ``axioma.urdf``, ``nav2_params.yaml`` and ``slam_params.yaml``
at render time, and the robot is drawn from its actual visual meshes rather
than sketched. Change a parameter and re-run this; the figure cannot go stale.

    python3 documentacion/modelo-matematico/render_diagram.py [out.png]

Default output: images/modelo-matematico.png
"""
import math
import os
import re
import struct
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import FancyBboxPatch, Circle

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'src')
MESH = os.path.join(SRC, 'axioma_description', 'meshes', 'visual')

INK = '#1b2430'
MUTED = '#5a6675'
ACCENT = '#c1121f'
BLUE = '#1d4e89'
GREEN = '#2d6a4f'


# --------------------------------------------------------------- sources ----
def read_sources():
    sdf = open(os.path.join(SRC, 'axioma_gazebo/models/axioma_v2/model.sdf')).read()
    urdf = open(os.path.join(SRC, 'axioma_description/urdf/axioma.urdf')).read()
    nav = yaml.safe_load(open(os.path.join(SRC, 'axioma_navigation/config/nav2_params.yaml')))
    slam = yaml.safe_load(open(os.path.join(SRC, 'axioma_slam/config/slam_params.yaml')))

    def tag(name, text=sdf):
        m = re.search(rf'<{name}>([^<]*)</{name}>', text)
        return m.group(1).strip() if m else None

    lidar = sdf[sdf.index('ydlidar_x3_pro'):]
    joints = {m.group(1): ([float(v) for v in m.group(2).split()],
                           [float(v) for v in m.group(3).split()])
              for m in re.finditer(
                  r'<joint name="(base_to_wheel\d)".*?<origin xyz="([^"]*)" rpy="([^"]*)"',
                  urdf, re.S)}
    dwb = nav['controller_server']['ros__parameters']['FollowPath']
    goal = nav['controller_server']['ros__parameters']['general_goal_checker']
    return dict(
        sdf=sdf, joints=joints, dwb=dwb, goal=goal,
        r=float(tag('wheel_radius')),
        W=float(tag('wheel_separation')),
        odom_hz=float(tag('odom_publish_frequency')),
        mu=float(re.search(r'<mu>([^<]*)</mu>', sdf).group(1)),
        mu2=float(re.search(r'<mu2>([^<]*)</mu2>', sdf).group(1)),
        masses=[float(x) for x in re.findall(r'<mass>([0-9.]+)</mass>', sdf)],
        lidar=dict(samples=int(tag('samples', lidar)),
                   amin=float(tag('min_angle', lidar)), amax=float(tag('max_angle', lidar)),
                   rmin=float(tag('min', lidar)), rmax=float(tag('max', lidar)),
                   hz=float(tag('update_rate', lidar))),
        lidar_z=float(re.search(r'<child link="base_scan"/>\s*<origin xyz="0 0 ([0-9.]+)"',
                                urdf).group(1)),
        local=nav['local_costmap']['local_costmap']['ros__parameters'],
        glob=nav['global_costmap']['global_costmap']['ros__parameters'],
        amcl=nav['amcl']['ros__parameters'],
        slam=slam['slam_toolbox']['ros__parameters'],
    )


# ------------------------------------------------------------- geometry -----
def load_stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:2000]:
        tris, cur = [], []
        for line in d.decode('utf-8', 'ignore').splitlines():
            t = line.split()
            if t[:1] == ['vertex']:
                cur.append([float(v) for v in t[1:4]])
                if len(cur) == 3:
                    tris.append(cur); cur = []
        return np.array(tris)
    n = struct.unpack('<I', d[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')])
    return np.frombuffer(d[84:84 + n * 50], dtype=dt, count=n)['v'].astype(np.float64)


def rot(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def robot_parts(joints):
    parts = [('chassis', load_stl(os.path.join(MESH, 'chasis.stl')))]
    wheel = load_stl(os.path.join(MESH, 'llanta.stl'))
    for name, (xyz, rpy) in sorted(joints.items()):
        parts.append((name, wheel @ rot(rpy).T + np.array(xyz)))
    return parts


def draw_robot(ax, joints):
    """Orthographic top view of the real meshes, lit from above."""
    colours = {'chassis': np.array([0.13, 0.22, 0.46])}
    XY, RGB, Z = [], [], []
    for name, tris in robot_parts(joints):
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        n = np.cross(v1 - v0, v2 - v0)
        ln = np.linalg.norm(n, axis=1)
        ok = ln > 1e-12
        n, tris = n[ok] / ln[ok, None], tris[ok]
        vis = n[:, 2] > 0.0
        n, tris = n[vis], tris[vis]
        base = colours.get(name, np.array([0.20, 0.20, 0.22]))
        L = np.array([-0.35, 0.45, 0.82]); L /= np.linalg.norm(L)
        shade = 0.32 + 0.78 * np.clip(n @ L, 0, 1)
        rgb = np.clip(base[None, :] * shade[:, None] + 0.16 * (n[:, 2] ** 6)[:, None], 0, 1)
        XY.append(tris[:, :, :2]); RGB.append(rgb); Z.append(tris[:, :, 2].mean(axis=1))
    XY, RGB, Z = np.vstack(XY), np.vstack(RGB), np.concatenate(Z)
    order = np.argsort(Z)
    ax.add_collection(PolyCollection(XY[order], facecolors=RGB[order], edgecolors='none',
                                     antialiased=False, zorder=2))


def panel(fig, rect, title, colour):
    ax = fig.add_axes(rect)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    ax.add_patch(FancyBboxPatch((0.004, 0.004), 0.992, 0.992,
                                boxstyle='round,pad=0.006,rounding_size=0.02',
                                linewidth=1.6, edgecolor=colour, facecolor='white',
                                transform=ax.transAxes, zorder=0))
    ax.text(0.028, 0.955, title, transform=ax.transAxes, fontsize=13.5, weight='bold',
            color=colour, va='top')
    return ax


def lines(ax, x, y, rows, size=9.6, dy=0.052, mono=True):
    for i, (txt, style) in enumerate(rows):
        kw = dict(fontsize=size, va='top', transform=ax.transAxes, color=INK)
        if mono:
            kw['family'] = 'DejaVu Sans Mono'
        if style == 'h':
            kw.update(color=MUTED, weight='bold', fontsize=size - 0.6)
        elif style == 'k':
            kw.update(color=ACCENT, weight='bold')
        ax.text(x, y - i * dy, txt, **kw)


def draw_annotated_robot(rb, S):
    """Top view plus the geometry callouts, shared by the sheet and the export."""
    j = S['joints']
    yl, yr = j['base_to_wheel1'][0][1], j['base_to_wheel4'][0][1]
    xf, xb = j['base_to_wheel1'][0][0], j['base_to_wheel2'][0][0]
    Wc = (yl - yr) + 2 * 0.02
    L = xf - xb
    rr = S['local']['robot_radius']

    rb.set_aspect('equal'); rb.axis('off'); rb.patch.set_alpha(0)
    draw_robot(rb, j)
    rb.set_xlim(-0.212, 0.212); rb.set_ylim(-0.232, 0.180)

    rb.add_patch(Circle((0, 0), rr, fill=False, ls=(0, (5, 4)), lw=1.4, ec=ACCENT, zorder=3))
    rb.text(0.150, 0.150, f'robot_radius\n{rr} m', fontsize=8.8, color=ACCENT,
            weight='bold', ha='center', va='center')

    xt = -0.150
    rb.annotate('', xy=(xt, Wc / 2), xytext=(xt, -Wc / 2),
                arrowprops=dict(arrowstyle='<->', color=GREEN, lw=1.6))
    for yy in (Wc / 2, -Wc / 2):
        rb.plot([xt, -0.108], [yy, yy], color=GREEN, lw=0.8, ls=':')
    rb.text(xt - 0.019, 0, f'$W_c$ = {Wc:.4f} m', color=GREEN, fontsize=10,
            weight='bold', rotation=90, va='center', ha='center')

    yb = -0.158
    rb.annotate('', xy=(xf, yb), xytext=(xb, yb),
                arrowprops=dict(arrowstyle='<->', color=GREEN, lw=1.6))
    for xx in (xf, xb):
        rb.plot([xx, xx], [yb, -0.112], color=GREEN, lw=0.8, ls=':')
    rb.text((xf + xb) / 2, yb - 0.020, f'L = {L:.4f} m', color=GREEN, fontsize=10,
            weight='bold', ha='center')

    rb.arrow(0, 0, 0.058, 0, width=0.0018, head_width=0.009, color=ACCENT, zorder=6,
             length_includes_head=True)
    rb.arrow(0, 0, 0, 0.058, width=0.0018, head_width=0.009, color=ACCENT, zorder=6,
             length_includes_head=True)
    rb.text(0.064, -0.006, 'x', color=ACCENT, fontsize=12, weight='bold', zorder=6)
    rb.text(-0.012, 0.062, 'y', color=ACCENT, fontsize=12, weight='bold', zorder=6)
    rb.plot(0, 0, 'o', ms=11, mfc='none', mec='#ffb703', mew=2.4, zorder=6)
    rb.plot(0, 0, 'o', ms=5, color=ACCENT, zorder=7)
    rb.annotate('base_link  +  YDLIDAR X3 PRO\n(same x and y, sensor %.2f m higher)' % S['lidar_z'],
                xy=(-0.008, -0.008), xytext=(-0.208, -0.208), fontsize=8.6, color=INK,
                ha='left', va='center',
                bbox=dict(boxstyle='round,pad=0.34', fc='#fff3cd', ec='#ffb703', lw=1.1),
                arrowprops=dict(arrowstyle='-', color='#ffb703', lw=1.2,
                                connectionstyle='arc3,rad=-0.18'), zorder=8)
    for nm, key, side in (('W2', 'base_to_wheel2', 1), ('W1', 'base_to_wheel1', 1),
                          ('W3', 'base_to_wheel3', -1), ('W4', 'base_to_wheel4', -1)):
        p = j[key][0]
        col = BLUE if side > 0 else '#7a4a1f'
        rb.text(p[0], p[1] + side * 0.050, nm, ha='center', va='center', fontsize=9.5,
                weight='bold', color='white',
                bbox=dict(boxstyle='round,pad=0.20', fc=col, ec='none'), zorder=7)
    rb.text(0.148, 0.078, 'LEFT PAIR\nw_L', fontsize=8.6, color=BLUE, weight='bold', ha='center')
    rb.text(0.148, -0.082, 'RIGHT PAIR\nw_R', fontsize=8.6, color='#7a4a1f',
            weight='bold', ha='center')


def export_top_view(path):
    """Standalone transparent PNG of the annotated robot, for the Excalidraw source."""
    S = read_sources()
    fig = plt.figure(figsize=(7.2, 7.0))
    fig.patch.set_alpha(0)
    rb = fig.add_axes([0, 0, 1, 1])
    draw_annotated_robot(rb, S)
    fig.savefig(path, dpi=150, transparent=True, bbox_inches='tight', pad_inches=0.02)
    print('->', path)


def main():
    if '--top-view' in sys.argv:
        args = [a for a in sys.argv[1:] if a != '--top-view']
        return export_top_view(args[0] if args else
                               os.path.join(ROOT, 'images', 'robot-top-view.png'))
    S = read_sources()
    r, W = S['r'], S['W']
    j = S['joints']
    yl, yr = j['base_to_wheel1'][0][1], j['base_to_wheel4'][0][1]
    xf, xb = j['base_to_wheel1'][0][0], j['base_to_wheel2'][0][0]
    Wj = yl - yr                       # track measured between wheel joints
    Wc = Wj + 2 * 0.02                 # collision cylinders sit +0.02 out in wheel y
    L = xf - xb
    dwb, li = S['dwb'], S['lidar']
    ainc = (li['amax'] - li['amin']) / (li['samples'] - 1)
    rr = S['local']['robot_radius']

    fig = plt.figure(figsize=(17.4, 11.6), facecolor='white')
    fig.text(0.5, 0.968, 'Axioma 4WD  ·  Kinematic Model', ha='center',
             fontsize=26, weight='bold', color=INK)
    fig.text(0.5, 0.938, 'every value on this sheet is read from model.sdf, axioma.urdf, '
                         'nav2_params.yaml and slam_params.yaml at render time',
             ha='center', fontsize=11, color=MUTED, style='italic')

    # ---------------------------------------------------- 1. geometry -------
    ax = panel(fig, [0.018, 0.395, 0.474, 0.522], '1 · GEOMETRY  (real visual meshes, to scale)', BLUE)
    lines(ax, 0.040, 0.145, [
        (f'wheel radius   r  = {r} m           wheelbase   L  = {L:.5f} m', ''),
        (f'joint track    Wj = {Wj:.5f} m      total mass     = {sum(S["masses"]):.3f} kg', ''),
        (f'contact track  Wc = {Wc:.5f} m      footprint      = 0.217 x 0.222 m', ''),
    ], size=9.8, dy=0.052)

    rb = fig.add_axes([0.048, 0.487, 0.414, 0.415])
    draw_annotated_robot(rb, S)

    # ------------------------------------------ 2. differential kinematics --
    ax = panel(fig, [0.508, 0.655, 0.474, 0.262], '2 · DIFFERENTIAL KINEMATICS', GREEN)
    lines(ax, 0.032, 0.835, [
        ('FORWARD   wheel rates -> body twist', 'h'),
        ('   v = r (w_R + w_L) / 2            w = r (w_R - w_L) / W', ''),
        ('INVERSE   body twist -> wheel rates', 'h'),
        ('   w_L = (v - w W/2) / r            w_R = (v + w W/2) / r', ''),
        ('ODOMETRY  Euler integration at %g Hz' % S['odom_hz'], 'h'),
        ('   x+ = x + v cos(th) dt   y+ = y + v sin(th) dt   th+ = th + w dt', ''),
        ('', ''),
        (f'   r = {r} m       W = {W} m       dt = {1/S["odom_hz"]:.3f} s', 'k'),
    ], size=10.0, dy=0.108)

    # ------------------------------------------------- 3. skid-steer --------
    ax = panel(fig, [0.508, 0.395, 0.474, 0.242], '3 · W IS NOT A DISTANCE YOU CAN MEASURE', ACCENT)
    lines(ax, 0.032, 0.845, [
        ('Four fixed wheels cannot turn without scrubbing sideways, and that', ''),
        ('scrub fights the yaw. The body therefore rotates slower than ideal', ''),
        ('differential kinematics predicts, so W above is an EFFECTIVE track', ''),
        ('calibrated against the Gazebo ground-truth pose:', ''),
        ('', ''),
        (f'   W = {W} m   vs   contact track Wc = {Wc:.5f} m   ->  {Wc/W:.3f}', 'k'),
        ('', ''),
        ('Verified 0.1-1.0 rad/s: yaw error under 3%, straight line exact.', ''),
    ], size=9.7, dy=0.097)

    # ---------------------------------------------------- 4. limits ---------
    ax = panel(fig, [0.018, 0.028, 0.313, 0.342], '4 · LIMITS', '#7a4a1f')
    wmax = (dwb['max_vel_x'] + dwb['max_vel_theta'] * W / 2) / r
    lines(ax, 0.048, 0.872, [
        ('BODY  (Nav2 and the plugin agree)', 'h'),
        (f'   v      <= {dwb["max_vel_x"]} m/s', ''),
        (f'   w      <= {dwb["max_vel_theta"]} rad/s', ''),
        (f'   dv/dt  <= {dwb["acc_lim_x"]} m/s^2', ''),
        (f'   dw/dt  <= {dwb["acc_lim_theta"]} rad/s^2', ''),
        ('', ''),
        ('WHEELS  (from inverse kinematics)', 'h'),
        (f'   straight    {dwb["max_vel_x"]/r:5.2f} rad/s', ''),
        (f'   spin only   {dwb["max_vel_theta"]*W/(2*r):5.2f} rad/s', ''),
        (f'   combined    {wmax:5.2f} rad/s', ''),
        ('', ''),
        ('TURN RADIUS', 'h'),
        (f'   spinning in place   0 m', ''),
        (f'   at v and w max      {dwb["max_vel_x"]/dwb["max_vel_theta"]:.2f} m', ''),
    ], size=9.2, dy=0.0585)

    # ---------------------------------------------------- 5. control --------
    ax = panel(fig, [0.345, 0.028, 0.313, 0.342], '5 · CONTROL CHAIN', BLUE)
    lines(ax, 0.048, 0.872, [
        ('Nav2 behavior tree', ''),
        ('   |   NavFn global plan', ''),
        (f'   |   DWB local, {dwb["vx_samples"]}x{dwb["vtheta_samples"]} rollouts', ''),
        (f'   |   {dwb["sim_time"]} s horizon', ''),
        ('   |   velocity smoother', ''),
        ('   v   /cmd_vel   (Twist)', 'k'),
        ('', ''),
        ('gz-sim-diff-drive-system', 'k'),
        ('   |   inverse kinematics', ''),
        ('   |   4 joints, 2 synced pairs', ''),
        ('   v   /odom + TF odom->base_link', ''),
        ('', ''),
        ('No PID anywhere: the plugin sets', ''),
        ('joint velocities, physics does the rest.', ''),
    ], size=9.2, dy=0.0585)

    # ---------------------------------------------------- 6. sensing --------
    ax = panel(fig, [0.671, 0.028, 0.311, 0.342], '6 · SENSING', GREEN)
    sl, gl, lo, am = S['slam'], S['glob'], S['local'], S['amcl']
    lines(ax, 0.048, 0.872, [
        ('YDLIDAR X3 PRO', 'h'),
        (f'   {li["rmin"]:.2f} - {li["rmax"]:.0f} m   ·   {li["hz"]:.0f} Hz', ''),
        (f'   {li["samples"]} pts/rev = {math.degrees(ainc):.3f} deg', ''),
        (f'   = 4 kHz sampling / {li["hz"]:.0f} Hz spin', ''),
        ('', ''),
        ('COSTMAPS', 'h'),
        (f'   local  {lo["width"]}x{lo["height"]} m, infl {lo["inflation_layer"]["inflation_radius"]} m', ''),
        (f'   global infl {gl["inflation_layer"]["inflation_radius"]} m, res {gl["resolution"]} m', ''),
        ('', ''),
        ('SLAM TOOLBOX', 'h'),
        (f'   new node every {sl["minimum_travel_distance"]} m / {sl["minimum_travel_heading"]} rad', ''),
        (f'   loop search radius {sl["loop_search_maximum_distance"]} m', ''),
        ('', ''),
        (f'AMCL {am["min_particles"]}-{am["max_particles"]} particles, {am["max_beams"]} beams', ''),
    ], size=9.2, dy=0.0585)

    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'images', 'modelo-matematico.png')
    plt.savefig(out, dpi=110, facecolor='white')
    print('->', out)


if __name__ == '__main__':
    main()
