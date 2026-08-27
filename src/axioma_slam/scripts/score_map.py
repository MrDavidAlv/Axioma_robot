#!/usr/bin/env python3
"""Score a SLAM map against the exact floor plan of the simulated world.

The world and an exact occupancy grid of it are both emitted by
``axioma_gazebo/scripts/generate_office_world.py``, so the plan in
``axioma_navigation/maps/ground_truth.pgm`` is ground truth by construction.
This compares the map SLAM Toolbox is currently publishing on ``/map`` against
it and reports two numbers:

  precision  how far every occupied cell of the SLAM map is from the nearest
             real obstacle, i.e. how much of the map is invented
  recall     of the real obstacles that fall inside explored territory, how
             many the SLAM map actually marks

Run it with the SLAM stack up (``ros2 launch axioma_bringup slam_bringup.launch.py``)
after driving the robot around, or pass ``--saved`` to score
``axioma_navigation/maps/mapa.pgm`` offline instead:

    python3 src/axioma_slam/scripts/score_map.py [--saved] [out.png] ["title"]
"""
import os
import sys
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
TRUTH_PGM = os.path.join(SRC, 'axioma_navigation', 'maps', 'ground_truth.pgm')
TRUTH_YAML = os.path.join(SRC, 'axioma_navigation', 'maps', 'ground_truth.yaml')
RES = 0.05


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', f'{path} is not a binary PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return np.flipud(data)                      # row 0 becomes min y


def read_origin(path):
    for line in open(path):
        if line.startswith('origin:'):
            x, y = line.split('[')[1].split(']')[0].split(',')[:2]
            return float(x), float(y)
    raise RuntimeError(f'no origin in {path}')


def grab_slam_map():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from nav_msgs.msg import OccupancyGrid

    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
    latched.reliability = ReliabilityPolicy.RELIABLE

    class Grabber(Node):
        def __init__(self):
            super().__init__('map_scorer')
            self.msg = None
            self.create_subscription(OccupancyGrid, '/map',
                                     lambda m: setattr(self, 'msg', m), latched)

    rclpy.init()
    node = Grabber()
    deadline = time.time() + 10
    while time.time() < deadline and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = node.msg
    rclpy.shutdown()
    if msg is None:
        raise SystemExit('nothing published on /map - is the SLAM stack running?')
    grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                      msg.info.width)
    return (grid,
            (msg.info.origin.position.x, msg.info.origin.position.y),
            msg.info.resolution)


def load_saved_map():
    pgm = os.path.join(SRC, 'axioma_navigation', 'maps', 'mapa.pgm')
    yaml = os.path.join(SRC, 'axioma_navigation', 'maps', 'mapa.yaml')
    grid = read_pgm(pgm).astype(np.int16)
    # PGM greyscale back to occupancy: 0 is occupied, 254 free, 205 unknown
    occ = np.full(grid.shape, -1, dtype=np.int16)
    occ[grid == 254] = 0
    occ[grid == 0] = 100
    return occ, read_origin(yaml), RES


def main():
    args = [a for a in sys.argv[1:] if a != '--saved']
    truth = read_pgm(TRUTH_PGM)
    truth_origin = read_origin(TRUTH_YAML)
    if '--saved' in sys.argv:
        slam, slam_origin, slam_res = load_saved_map()
    else:
        slam, slam_origin, slam_res = grab_slam_map()

    # Both grids are resampled onto one common world-frame grid so that cells
    # can be compared directly even though the origins differ.
    ox = min(truth_origin[0], slam_origin[0]) - 0.5
    oy = min(truth_origin[1], slam_origin[1]) - 0.5
    width = int((max(truth_origin[0] + truth.shape[1] * RES,
                     slam_origin[0] + slam.shape[1] * slam_res) + 0.5 - ox) / RES)
    height = int((max(truth_origin[1] + truth.shape[0] * RES,
                      slam_origin[1] + slam.shape[0] * slam_res) + 0.5 - oy) / RES)

    def paste(mask, origin, res):
        out = np.zeros((height, width), bool)
        rows, cols = np.nonzero(mask)
        if len(rows) == 0:
            return out
        xs = origin[0] + (cols + 0.5) * res
        ys = origin[1] + (rows + 0.5) * res
        c = ((xs - ox) / RES).astype(int)
        r = ((ys - oy) / RES).astype(int)
        keep = (r >= 0) & (r < height) & (c >= 0) & (c < width)
        out[r[keep], c[keep]] = True
        return out

    truth_occ = paste(truth == 0, truth_origin, RES)
    truth_known = paste(truth != 205, truth_origin, RES)
    slam_occ = paste(slam > 65, slam_origin, slam_res)
    slam_known = paste(slam >= 0, slam_origin, slam_res)

    to_truth = distance_transform_edt(~truth_occ) * RES
    to_slam = distance_transform_edt(~slam_occ) * RES

    precision = to_truth[slam_occ & truth_known]
    recall_mask = truth_occ & slam_known
    recall = to_slam[recall_mask]

    print(f'occupied cells: SLAM={slam_occ.sum()}  ground truth={truth_occ.sum()}')
    print(f'PRECISION (SLAM cell -> nearest real obstacle): '
          f'mean={precision.mean():.3f} p50={np.percentile(precision, 50):.3f} '
          f'p95={np.percentile(precision, 95):.3f} max={precision.max():.3f} m')
    print(f'  within 0.10 m: {100 * (precision <= 0.10).mean():5.1f} %   '
          f'within 0.20 m: {100 * (precision <= 0.20).mean():5.1f} %   '
          f'beyond 0.50 m (invented): {100 * (precision > 0.50).mean():5.1f} %')
    print(f'RECALL (real obstacle -> nearest SLAM cell, explored area only): '
          f'mean={recall.mean():.3f} p95={np.percentile(recall, 95):.3f} m')
    print(f'  found within 0.10 m: {100 * (recall <= 0.10).mean():5.1f} %   '
          f'explored: {100 * recall_mask.sum() / truth_occ.sum():5.1f} % of real obstacles')
    score = 100 * (precision <= 0.10).mean() * (recall <= 0.10).mean()
    print(f'\nSCORE (precision@10cm x recall@10cm) = {score:.1f} / 100')

    if args:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        rgb = np.ones((height, width, 3))
        rgb[~slam_known] = 0.75
        rgb[truth_occ] = [1.0, 0.55, 0.55]
        rgb[slam_occ] = [0.0, 0.0, 0.0]
        rgb[slam_occ & (to_truth > 0.25)] = [0.0, 0.4, 1.0]
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.imshow(rgb, origin='lower',
                  extent=[ox, ox + width * RES, oy, oy + height * RES])
        title = args[1] if len(args) > 1 else 'SLAM map vs ground truth'
        ax.set_title(f'{title}\nscore {score:.1f}/100  -  pink: real geometry, '
                     f'black: SLAM map, blue: further than 25 cm from any wall')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(alpha=.25)
        plt.tight_layout()
        plt.savefig(args[0], dpi=95)
        print('->', args[0])


if __name__ == '__main__':
    main()
