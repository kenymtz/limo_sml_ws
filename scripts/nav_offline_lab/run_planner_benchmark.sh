#!/bin/bash
# NAV-OFFLINE-R2 — ejecuta el planner Nav2 REAL offline (dominio 99, localhost) y el benchmark.
# Uso: run_planner_benchmark.sh <CONFIG> [audit-only|full]
#   CONFIG -> config/offline_lab/planner_offline_<CONFIG>.yaml
#   Opcional: MAP_YAML=<yaml validado> (def. mapa sintético)  RUN_TAG=<sufijo salida> (def. CONFIG)
# Levanta SOLO config/offline_lab/planner_offline.launch.py (map_server, planner_server,
# 2 TF estáticas, lifecycle_manager). Sin hardware, sin controller, sin bt_navigator, sin /cmd_vel.
set -u
F=${1:?CONFIG}; MODE=${2:-full}; TAG=${RUN_TAG:-$F}
WS="$HOME/limo_sml_ws"; LAB="$WS/results/navigation_offline_lab"; OUT="$LAB/planner_runs"
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99
MAP="${MAP_YAML:-$LAB/synthetic_arena/synthetic_arena.yaml}"
[ -f "$WS/config/offline_lab/planner_offline_$F.yaml" ] || { echo "no existe config $F"; exit 2; }
/usr/bin/python3 "$WS/scripts/nav_offline_lab/validate_ros_map.py" --map-yaml "$MAP" --strict --expect-resolution 0.03 >/dev/null \
  || { echo "mapa sintético no valida"; exit 2; }

setsid ros2 launch "$WS/config/offline_lab/planner_offline.launch.py" map:="$MAP" \
  params_file:="$WS/config/offline_lab/planner_offline_$F.yaml" > "$OUT/launch_$TAG.log" 2>&1 &
LPID=$!
cleanup() { kill -INT -- -$LPID 2>/dev/null; sleep 3; kill -TERM -- -$LPID 2>/dev/null; wait $LPID 2>/dev/null; }
trap cleanup EXIT
sleep 8
C="$WS/scripts/nav_offline_lab/planner_benchmark_client.py"
/usr/bin/python3 "$C" audit --out "$OUT/audit_$TAG.json"; ARC=$?
[ $ARC -eq 0 ] || { echo "AUDITORÍA: elementos de movimiento detectados o error (rc=$ARC). Abortando."; exit 3; }
[ "$MODE" = "audit-only" ] && exit 0
/usr/bin/python3 "$C" run --pairs "$LAB/synthetic_benchmark_pairs.json" --out "$OUT/paths_$TAG.json" --config "$TAG"
