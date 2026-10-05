#!/bin/bash
# Experimento de procedencia de free_thresh (OFFLINE, AISLADO dominio 99, localhost).
# 1) publica un OccupancyGrid sintético con TODOS los valores -1,0..100 en /offline_prov/map
# 2) lo guarda con map_saver_cli: (a) defaults, (b) --free 0.196, (c) --free 0.10
# 3) reporta yaml escrito y mapeo valor_celda -> pixel
# 4) lista parámetros declarados por nav2 map_server (¿existe 'free_thresh'?)
# Sin hardware. Solo nodos de mapa. Salida en $OUT (por defecto results/navigation_offline_lab/provenance/)
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99
OUT=${1:-$HOME/limo_sml_ws/results/navigation_offline_lab/provenance}
mkdir -p "$OUT"
PY=/usr/bin/python3

$PY - <<'PY' &
import rclpy, numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node('offline_prov_grid_pub')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
p = n.create_publisher(OccupancyGrid, '/offline_prov/map', qos)
m = OccupancyGrid(); m.header.frame_id = 'map'
vals = np.array([-1] + list(range(0, 101)), dtype=np.int8)       # 102 valores, 1 por columna
m.info.resolution = 0.05; m.info.width = len(vals); m.info.height = 4
m.info.origin.orientation.w = 1.0
m.data = np.tile(vals, 4).tolist()
p.publish(m)
import time; t = time.time()
while time.time() - t < 40: rclpy.spin_once(n, timeout_sec=0.5)
PY
PUB=$!
trap "kill $PUB 2>/dev/null; wait $PUB 2>/dev/null" EXIT
sleep 2
for tag in default free0196 free010; do
  case $tag in default) EXTRA="";; free0196) EXTRA="--free 0.196";; free010) EXTRA="--free 0.10";; esac
  timeout 20 /opt/ros/humble/lib/nav2_map_server/map_saver_cli -t /offline_prov/map -f "$OUT/prov_$tag" $EXTRA \
     --ros-args -r __node:=offline_prov_saver_$tag > "$OUT/prov_$tag.log" 2>&1
  echo "saver $tag rc=$?"
done

# parámetros declarados por map_server
/opt/ros/humble/lib/nav2_map_server/map_server --ros-args -r __node:=offline_prov_ms \
   -p yaml_filename:=$OUT/prov_default.yaml -p topic_name:=/offline_prov/ms_map >/dev/null 2>&1 &
MS=$!
sleep 3
$PY - <<'PY' > "$OUT/map_server_declared_params.txt"
import rclpy
from rcl_interfaces.srv import ListParameters
rclpy.init(); n = rclpy.create_node('offline_prov_param_reader')
c = n.create_client(ListParameters, '/offline_prov_ms/list_parameters')
c.wait_for_service(timeout_sec=10)
f = c.call_async(ListParameters.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
print('\n'.join(sorted(f.result().result.names)))
PY
kill $MS 2>/dev/null; wait $MS 2>/dev/null

$PY - "$OUT" <<'PY'
import sys, os, json, numpy as np
sys.path.insert(0, os.path.expanduser('~/limo_sml_ws/scripts/nav_offline_lab'))
from common import read_pgm
out = sys.argv[1]; res = {}
vals = [-1] + list(range(0, 101))
for tag in ('default', 'free0196', 'free010'):
    y = os.path.join(out, f'prov_{tag}.yaml')
    if not os.path.exists(y):
        res[tag] = 'NO GENERADO'; continue
    arr = read_pgm(os.path.join(out, f'prov_{tag}.pgm'))['arr']
    row = arr[0]  # ojo: map_saver invierte eje y; todas las filas son iguales
    groups = {}
    for v, px in zip(vals, row):
        groups.setdefault(int(px), []).append(v)
    res[tag] = {'yaml': open(y).read().strip().splitlines(),
                'pixel_to_cell_values': {px: (f'{g[0]}..{g[-1]}' if len(g) > 1 else str(g[0])) + f' ({len(g)} valores)'
                                         for px, g in sorted(groups.items())}}
res['map_server_declared_params'] = open(os.path.join(out, 'map_server_declared_params.txt')).read().split()
json.dump(res, open(os.path.join(out, 'provenance_experiment.json'), 'w'), indent=1)
print(json.dumps(res, indent=1))
PY
