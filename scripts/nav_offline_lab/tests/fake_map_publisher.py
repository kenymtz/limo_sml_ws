#!/usr/bin/python3
"""SOLO PARA TESTS OFFLINE (dominio aislado). Publica el mapa sintético como
OccupancyGrid (transient_local) en --topic. Añade celdas con probabilidades
intermedias (10, 22, 30, 50, 64, 65, 80) para ejercitar los umbrales del guardado.
--changing: republica cada 0.3 s con una celda distinta (simula SLAM activo)."""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from validate_ros_map import read_image  # noqa

ap = argparse.ArgumentParser()
ap.add_argument('--topic', default='/map')
ap.add_argument('--changing', action='store_true')
ap.add_argument('--duration', type=float, default=120)
a = ap.parse_args()

arr, _ = read_image(os.path.expanduser(
    '~/limo_sml_ws/results/navigation_offline_lab/synthetic_arena/synthetic_arena.pgm'))
g = np.flipud(arr)
grid = np.where(g == 254, 0, np.where(g == 0, 100, -1)).astype(np.int8)
for k, v in enumerate((10, 22, 30, 50, 64, 65, 80)):
    grid[100 + 5 * k, 100:110] = v

rclpy.init()
n = rclpy.create_node('offline_fake_map_pub')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
p = n.create_publisher(OccupancyGrid, a.topic, qos)


def msg(i):
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    m.info.resolution = 0.03
    m.info.width, m.info.height = grid.shape[1], grid.shape[0]
    m.info.origin.position.x = m.info.origin.position.y = -0.81
    m.info.origin.orientation.w = 1.0
    gg = grid.copy()
    if a.changing:
        gg[200, 50 + (i % 200)] = 100
    m.data = gg.ravel().tolist()
    return m


p.publish(msg(0))
t0, i = time.time(), 0
while time.time() - t0 < a.duration:
    rclpy.spin_once(n, timeout_sec=0.3)
    if a.changing:
        i += 1
        p.publish(msg(i))
