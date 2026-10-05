#!/bin/bash
# NAV-OFFLINE-R4-A — DWB real offline (dominio 99, localhost). Sin hardware, sin FollowPath, sin /cmd_vel.
# Uso:
#   run_dwb_r4a.sh server <PARAMS_YAML> <TAG>                    verificación DWB CONFIG (controller_server real)
#   run_dwb_r4a.sh eval   <PARAMS_YAML> <TAG> <CASES_TXT>        evaluación de casos con dwb_offline_eval
# Opcional: MAP_YAML=<yaml validado> (def. mapa sintético R2/R3)  OUT_SUBDIR=<dir bajo navigation_offline_lab> (def. dwb_r4a)
#           SETTLE_MS=<espera tras recentrar costmap> (def. 300)
set -u
MODE=${1:?mode}; PARAMS=${2:?params}; TAG=${3:?tag}; CASES=${4:-}
WS="$HOME/limo_sml_ws"; LAB="$WS/results/navigation_offline_lab"; OUT="$LAB/${OUT_SUBDIR:-dwb_r4a}"
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99
MAP="${MAP_YAML:-$LAB/synthetic_arena/synthetic_arena.yaml}"
/usr/bin/python3 "$WS/scripts/nav_offline_lab/validate_ros_map.py" --map-yaml "$MAP" --strict --expect-resolution 0.03 >/dev/null \
  || { echo "mapa no valida"; exit 2; }
# nada debe estar corriendo en el dominio 99
if [ -n "$(ros2 node list 2>/dev/null)" ]; then echo "dominio 99 no está vacío; abortando"; ros2 node list; exit 2; fi

LA=(mode:=$MODE map:="$MAP" params_file:="$PARAMS")
[ "$MODE" = eval ] && LA+=(cases_file:="$CASES" out_file:="$OUT/eval_$TAG.jsonl" settle_ms:="${SETTLE_MS:-300}")
SP="${SIM_PARAMS:-}"; [ -z "$SP" ] && SP='{}'
[ "$MODE" = closedloop ] && LA+=(cases_file:="$CASES" out_file:="$OUT/sim_$TAG.jsonl" sim_params:="$SP")
setsid ros2 launch "$WS/config/offline_lab/dwb_offline.launch.py" "${LA[@]}" > "$OUT/launch_$TAG.log" 2>&1 &
LPID=$!
cleanup() { kill -INT -- -$LPID 2>/dev/null; sleep 4; kill -TERM -- -$LPID 2>/dev/null; wait $LPID 2>/dev/null; }
trap cleanup EXIT
sleep 10
/usr/bin/python3 "$WS/scripts/nav_offline_lab/dwb_r4a_audit.py" --mode "$MODE" --out "$OUT/audit_$TAG.json"; ARC=$?
if [ $ARC -ne 0 ]; then echo "AUDITORÍA FALLÓ (rc=$ARC). Abortando."; exit 3; fi

if [ "$MODE" = server ]; then
  {
    echo "== lifecycle"; for nd in /map_server /controller_server /local_costmap/local_costmap; do
      echo "$nd: $(ros2 lifecycle get $nd 2>&1)"; done
    echo "== params FollowPath"
    for p in FollowPath.plugin FollowPath.critics FollowPath.holonomic FollowPath.max_vel_x FollowPath.max_vel_theta \
             FollowPath.acc_lim_x FollowPath.acc_lim_theta FollowPath.vx_samples FollowPath.vy_samples \
             FollowPath.vtheta_samples FollowPath.sim_time FollowPath.short_circuit_trajectory_evaluation \
             FollowPath.BaseObstacle.scale FollowPath.PathAlign.scale FollowPath.GoalAlign.scale \
             FollowPath.PathDist.scale FollowPath.GoalDist.scale controller_frequency; do
      echo "$p = $(ros2 param get /controller_server $p 2>&1 | tr '\n' ' ')"; done
    echo "== params local_costmap"
    for p in footprint footprint_padding plugins resolution width height global_frame rolling_window; do
      echo "$p = $(ros2 param get /local_costmap/local_costmap $p 2>&1 | tr '\n' ' ')"; done
    echo "== sink"; ros2 topic info /offline_lab/cmd_vel_sink 2>&1
  } > "$OUT/server_check_$TAG.txt"
  cat "$OUT/server_check_$TAG.txt"
  exit 0
fi

# eval: esperar a que dwb_offline_eval termine (sale solo al procesar todos los casos)
T0=$(date +%s)
while pgrep -f "lib/limo_nav_offline_lab/dwb_(offline_eval|closed_loop_sim)" >/dev/null; do
  sleep 5
  [ $(( $(date +%s) - T0 )) -gt 7200 ] && { echo "timeout"; exit 4; }
done
F="$OUT/eval_$TAG.jsonl"; [ "$MODE" = closedloop ] && F="$OUT/sim_$TAG.jsonl"
echo "$MODE terminado en $(( $(date +%s) - T0 )) s; líneas: $(wc -l < "$F")"
