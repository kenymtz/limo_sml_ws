#!/bin/bash
# Verifica cómo nav2_map_server interpreta un map.yaml y audita el grafo ROS.
# AISLADO: ROS_LOCALHOST_ONLY=1 + ROS_DOMAIN_ID=99. Solo lanza nav2_map_server
# (publica un OccupancyGrid); sin hardware, sin riesgo de movimiento. No modifica el mapa.
# Uso: verify_map_server_load.sh <map.yaml>   -> imprime JSON (valores + grafo)
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99
YAML=${1:?uso: $0 map.yaml}
/opt/ros/humble/lib/nav2_map_server/map_server --ros-args -r __node:=offline_map_check \
  -p yaml_filename:=$YAML -p topic_name:=/offline_map_check/map >/dev/null 2>&1 &
PID=$!
trap "kill $PID 2>/dev/null; wait $PID 2>/dev/null" EXIT
YAML=$YAML /usr/bin/python3 - <<'PY' 2>/dev/null
import json, os, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = rclpy.create_node('offline_map_reader')
out = {'yaml': os.environ['YAML'], 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
       'ros_localhost_only': os.environ.get('ROS_LOCALHOST_ONLY')}
cli = n.create_client(ChangeState, '/offline_map_check/change_state')
assert cli.wait_for_service(timeout_sec=15), 'map_server no respondió'
for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
    f = cli.call_async(ChangeState.Request(transition=Transition(id=tid)))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    assert f.result() and f.result().success, f'transición {tid} falló'
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/offline_map_check/map', lambda m: got.setdefault('m', m), qos)
t = time.time()
while 'm' not in got and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.5)
m = got['m']
d = np.array(m.data)
out['map'] = {'w': m.info.width, 'h': m.info.height, 'res': round(m.info.resolution, 4),
              'origin': [round(m.info.origin.position.x, 4), round(m.info.origin.position.y, 4)],
              'free_0': int((d == 0).sum()), 'occupied_100': int((d == 100).sum()),
              'unknown_-1': int((d == -1).sum()), 'other': int(((d != 0) & (d != 100) & (d != -1)).sum())}

# ---- auditoría del grafo (dominio 99) ----
time.sleep(1.0)
g = {'nodes': sorted(f'{ns.rstrip("/")}/{nm}' for nm, ns in n.get_node_names_and_namespaces())}
for nm, ns in n.get_node_names_and_namespaces():
    if nm != 'offline_map_check':
        continue
    g['map_server_publishers'] = sorted(f'{t} [{",".join(ty)}]' for t, ty in n.get_publisher_names_and_types_by_node(nm, ns))
    g['map_server_subscribers'] = sorted(t for t, _ in n.get_subscriber_names_and_types_by_node(nm, ns))
    g['map_server_services'] = sorted(s for s, _ in n.get_service_names_and_types_by_node(nm, ns))
topics = dict(n.get_topic_names_and_types(no_demangle=True))
g['all_topics'] = sorted(topics)
g['action_topics'] = sorted(t for t in topics if '/_action/' in t)
motion_kw = ('cmd_vel', 'joint', 'trajectory', 'controller', 'gripper', 'pump', 'twist')
g['motion_related_topics'] = sorted(t for t in topics if any(k in t.lower() for k in motion_kw))
g['non_map_publishers'] = sorted(t for t in g.get('map_server_publishers', [])
                                 if not t.startswith(('/offline_map_check/map ', '/rosout ', '/parameter_events ',
                                                      '/offline_map_check/transition_event ')))
out['graph'] = g
print(json.dumps(out, indent=1))
PY
