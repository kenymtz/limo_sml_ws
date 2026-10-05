#!/usr/bin/python3
"""FASE 6/7 — cliente del planner Nav2 REAL (ComputePathToPose). SOLO dominio 99 / localhost.

  audit  --out audit.json      grafo ROS (nodos, topics, acciones) + sonda del global costmap
  run    --pairs pairs.json --out paths.json [--timeout 15]

No envía NavigateToPose ni FollowPath; no publica nada (solo acción de planificación).
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '99':
    sys.exit('requiere ROS_LOCALHOST_ONLY=1 y ROS_DOMAIN_ID=99')

MOTION_KW = ('cmd_vel', 'joint', 'trajectory', 'controller', 'gripper', 'pump', 'twist', 'follow_path',
             'navigate_to_pose', 'navigate_through_poses', 'spin', 'backup', 'drive_on_heading')


def pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
    return p


def cmd_audit(n, a):
    time.sleep(2.0)
    topics = dict(n.get_topic_names_and_types())
    nodes = sorted(f'{ns.rstrip("/")}/{nm}' for nm, ns in n.get_node_names_and_namespaces())
    per_node = {}
    for nm, ns in n.get_node_names_and_namespaces():
        if nm.startswith('offline_bench') or nm.startswith('_ros2cli'):
            continue
        per_node[f'{ns.rstrip("/")}/{nm}'] = {
            'pub': sorted(t for t, _ in n.get_publisher_names_and_types_by_node(nm, ns)),
            'sub': sorted(t for t, _ in n.get_subscriber_names_and_types_by_node(nm, ns))}
    raw = dict(n.get_topic_names_and_types(no_demangle=True))
    actions = sorted({t.split('/_action/')[0].replace('rq/', '/').replace('rr/', '/').replace('rt/', '/')
                      for t in raw if '/_action/' in t})
    motion = sorted(t for t in list(topics) + actions if any(k in t.lower() for k in MOTION_KW))
    # sonda del costmap global
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.setdefault('m', m), qos)
    t = time.time()
    while 'm' not in got and time.time() - t < 15:
        rclpy.spin_once(n, timeout_sec=0.5)
    probe = None
    if 'm' in got:
        m = got['m']
        g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        np.savez_compressed(a.out.replace('.json', '_costmap.npz'), costmap=g, res=m.info.resolution,
                            origin=[m.info.origin.position.x, m.info.origin.position.y])
        ox, oy, r = m.info.origin.position.x, m.info.origin.position.y, m.info.resolution

        def at(x, y):
            return int(g[int((y - oy) / r), int((x - ox) / r)])
        v, c = np.unique(g, return_counts=True)
        probe = {'w': m.info.width, 'h': m.info.height, 'res': r, 'origin': [ox, oy],
                 'value_hist_top': {int(a_): int(b_) for a_, b_ in sorted(zip(v, c), key=lambda z: -z[1])[:8]},
                 'n_minus1_unknown': int((g == -1).sum()), 'n_100_lethal': int((g == 100).sum()),
                 'n_99_inscribed': int((g == 99).sum()),
                 'value_at_unknown_outside_walls(-0.5,-0.5)': at(-0.5, -0.5),
                 'value_at_unknown_outside_walls(10.6,5.0)': at(10.6, 5.0),
                 'value_at_open_free(3.5,5.5)': at(3.5, 5.5),
                 'value_N2_centerline(5.0,8.025)': at(5.0, 8.025),
                 'value_N1_centerline(5.0,5.025)': at(5.0, 5.025),
                 'value_WIDE_centerline(5.0,2.01)': at(5.0, 2.01)}
    # sonda del /map entregado por map_server (¿llega el -1?)
    gm_ = {}
    n.create_subscription(OccupancyGrid, '/map', lambda m: gm_.setdefault('m', m), qos)
    t = time.time()
    while 'm' not in gm_ and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.5)
    map_probe = None
    if 'm' in gm_:
        d = np.array(gm_['m'].data)
        map_probe = {'free_0': int((d == 0).sum()), 'occupied_100': int((d == 100).sum()),
                     'unknown_-1': int((d == -1).sum())}
    out = {'map_probe': map_probe, 'ros_domain_id': os.environ['ROS_DOMAIN_ID'], 'ros_localhost_only': os.environ['ROS_LOCALHOST_ONLY'],
           'nodes': nodes, 'per_node': per_node, 'topics': sorted(topics), 'actions': actions,
           'motion_related': motion, 'costmap_probe': probe}
    json.dump(out, open(a.out, 'w'), indent=1)
    print(json.dumps({k: out[k] for k in ('nodes', 'actions', 'motion_related', 'map_probe', 'costmap_probe')}, indent=1))
    return 0 if not motion else 3


def cmd_run(n, a):
    pairs = json.load(open(a.pairs))['pairs']
    ac = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=20):
        print('sin servidor /compute_path_to_pose'); return 2
    results = []
    for p in pairs:
        goal = ComputePathToPose.Goal()
        st = n.get_clock().now().to_msg()
        goal.start = pose(*p['start'], st); goal.goal = pose(*p['goal'], st)
        goal.use_start = True; goal.planner_id = 'GridBased'
        t0 = time.monotonic()
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=a.timeout)
        rec = dict(p)
        gh = fut.result()
        if gh is None or not gh.accepted:
            rec.update(status='REJECTED', path=[], planning_time_s=None, wall_time_s=time.monotonic() - t0)
            results.append(rec); continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=a.timeout)
        wall = time.monotonic() - t0
        if rf.result() is None:
            rec.update(status='TIMEOUT', path=[], planning_time_s=None, wall_time_s=wall)
            gh.cancel_goal_async()
        else:
            r = rf.result()
            path = [[ps.pose.position.x, ps.pose.position.y,
                     float(2 * np.arctan2(ps.pose.orientation.z, ps.pose.orientation.w))] for ps in r.result.path.poses]
            name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                    GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(r.status, str(r.status))
            pt = r.result.planning_time
            rec.update(status=name, path=path, planning_time_s=pt.sec + pt.nanosec * 1e-9, wall_time_s=wall)
        results.append(rec)
        print(f'{rec["test_id"]} {rec["category"]:<18} {rec["status"]:<9} poses={len(rec["path"]):4d} '
              f'plan={rec["planning_time_s"]} wall={wall:.3f}', flush=True)
    json.dump({'config': a.config, 'results': results}, open(a.out, 'w'))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    p = sp.add_parser('audit'); p.add_argument('--out', required=True)
    p = sp.add_parser('run'); p.add_argument('--pairs', required=True); p.add_argument('--out', required=True)
    p.add_argument('--timeout', type=float, default=15.0); p.add_argument('--config', default='')
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('offline_bench_client')
    try:
        return {'audit': cmd_audit, 'run': cmd_run}[a.cmd](n, a)
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
