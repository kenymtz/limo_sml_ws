#!/usr/bin/python3
"""NAV-OFFLINE-R4-A — auditoría del grafo ROS (dominio 99) durante la evaluación DWB.

FALLA (rc=3) si:
  - existe el topic /cmd_vel (cualquier tipo), o
  - existe un topic Twist/TwistStamped distinto de /offline_lab/cmd_vel_sink, o
  - /offline_lab/cmd_vel_sink tiene algún suscriptor, o
  - hay nodos fuera de la lista permitida para el modo.
Solo lee el grafo; no publica nada.
"""
import argparse
import json
import os
import sys
import time

import rclpy

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '99':
    sys.exit('requiere ROS_LOCALHOST_ONLY=1 y ROS_DOMAIN_ID=99')

ALLOWED = {
    'server': {'/map_server', '/controller_server', '/local_costmap/local_costmap', '/static_tf_map_odom',
               '/static_tf_odom_base', '/lifecycle_manager_offline'},
    # R4-G0: /global_costmap/global_costmap solo existe con plan_mode=replan_1hz (NavFn embebido; sin Twist/acciones)
    'closedloop': {'/map_server', '/dwb_closed_loop_sim', '/local_costmap/local_costmap', '/lifecycle_manager_offline',
                   '/global_costmap/global_costmap'},
    'eval': {'/map_server', '/dwb_offline_eval', '/local_costmap/local_costmap', '/static_tf_map_odom',
             '/lifecycle_manager_offline'},
}
TWIST = ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped')
SINK = '/offline_lab/cmd_vel_sink'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', required=True, choices=list(ALLOWED))
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('r4a_audit')
    time.sleep(3.0)
    nodes = sorted(f'{ns.rstrip("/")}/{nm}' for nm, ns in n.get_node_names_and_namespaces()
                   if nm != 'r4a_audit' and not nm.startswith('_ros2cli') and not nm.startswith('transform_listener'))
    topics = dict(n.get_topic_names_and_types())
    per_node = {}
    for nm, ns in n.get_node_names_and_namespaces():
        if nm == 'r4a_audit' or nm.startswith('_ros2cli'):
            continue
        per_node[f'{ns.rstrip("/")}/{nm}'] = {
            'pub': sorted(t for t, _ in n.get_publisher_names_and_types_by_node(nm, ns)),
            'sub': sorted(t for t, _ in n.get_subscriber_names_and_types_by_node(nm, ns))}
    raw = dict(n.get_topic_names_and_types(no_demangle=True))
    actions = sorted({t.split('/_action/')[0].replace('rq/', '/').replace('rr/', '/').replace('rt/', '/')
                      for t in raw if '/_action/' in t})
    twist_topics = {t: ty for t, ty in topics.items() if any(x in ty for x in TWIST)}
    sink_info = None
    if SINK in topics:
        sink_info = {'publishers': [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                                    for i in n.get_publishers_info_by_topic(SINK)],
                     'subscribers': [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                                     for i in n.get_subscriptions_info_by_topic(SINK)]}
    problems = []
    if '/cmd_vel' in topics:
        problems.append('existe /cmd_vel')
    for t in twist_topics:
        if t != SINK:
            problems.append(f'topic Twist inesperado: {t}')
    if sink_info and sink_info['subscribers']:
        problems.append(f'{SINK} tiene suscriptores: {sink_info["subscribers"]}')
    # ignora nodos internos transitorios (transform_listener_impl_*)
    extra = [x for x in nodes if x not in ALLOWED[a.mode] and 'transform_listener_impl' not in x]
    if extra:
        problems.append(f'nodos no permitidos: {extra}')
    res = dict(mode=a.mode, nodes=nodes, actions=actions, twist_topics=twist_topics, sink=sink_info,
               per_node=per_node, topics=sorted(topics), problems=problems, PASS=not problems)
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('mode', 'nodes', 'actions', 'twist_topics', 'sink', 'problems', 'PASS')},
                     indent=1))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if not problems else 3


if __name__ == '__main__':
    sys.exit(main())
