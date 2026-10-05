#!/bin/bash
# NAV-OFFLINE-R4-G0 — fidelidad de replanning: 25 casos representativos × {fixed_plan, replan_1hz, replan_same_1hz}.
# fixed_plan: dwb_offline_R4F_010_sim.yaml (candidata R4-F sin tocar). replan_*: dwb_offline_R4G0_replan_sim.yaml.
# Mismas semillas (líneas RUN/PATH copiadas de dwb_r4b/runs_<S>.txt), mismos SIM_PARAMS. debug_eval_runs en todas.
# Offline: run_dwb_r4a.sh fuerza ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99 y audita el grafo (sin Twist, sin acciones).
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4g0"; mkdir -p "$D/runs"
export OUT_SUBDIR=dwb_r4g0
/usr/bin/python3 "$WS/scripts/nav_offline_lab/verify_r4g0_configs.py" > "$D/config_verification.log" || { echo "verificación FALLÓ"; exit 2; }
declare -A SC=(
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
)
declare -A IDS=(
  [S1_nominal]='T041 T039 T045 T043 T023 T028 T025 T005 T015 T033 T050 T060'
  [S2_ruido_xy]='T045 T041 T029 T050'
  [S4_latencia_0p6]='T042 T045 T024 T040'
  [S5_combinado_medido]='T041 T023 T019 T033 T055'
)
for S in S1_nominal S2_ruido_xy S4_latencia_0p6 S5_combinado_medido; do
  RF="$D/runs/runs_$S.txt"
  /usr/bin/python3 - "$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt" "$RF" ${IDS[$S]} <<'EOF'
import sys
src, dst, ids = sys.argv[1], sys.argv[2], set(sys.argv[3:])
out, keep = [], False
for ln in open(src):
    if ln.startswith('RUN'):
        keep = ln.split()[1] in ids
    if keep:
        out.append(ln)
open(dst, 'w').writelines(out)
assert sum(l.startswith('RUN') for l in out) == len(ids), 'faltan casos'
EOF
  DBG=$(echo ${IDS[$S]} | tr ' ' ',')
  for MODE in fixed_plan replan_1hz replan_same_1hz; do
    CFG="$WS/config/offline_lab/dwb_offline_R4G0_replan_sim.yaml"; [ $MODE = fixed_plan ] && CFG="$WS/config/offline_lab/dwb_offline_R4F_010_sim.yaml"
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "
import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; d['debug_eval_runs']='$DBG'
if '$MODE'!='fixed_plan': d['plan_mode']='$MODE'
print(json.dumps(d))")
    SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$CFG" "${S}_${MODE}" "$RF" 2>&1 \
      | grep -E "terminado|FALL|abort|timeout|Abort|vacío|valida" | sed "s/^/$S $MODE: /"
    sleep 3
  done
done
echo TODO_TERMINADO
