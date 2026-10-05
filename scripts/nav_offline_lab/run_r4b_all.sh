#!/bin/bash
# NAV-OFFLINE-R4-B — todos los escenarios x {A: BaseObstacle, B: ObstacleFootprint} sobre los 64 paths R3.
# Sin hardware, sin /cmd_vel: usa run_dwb_r4a.sh closedloop (dominio 99, localhost, auditoría del grafo).
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4b"; mkdir -p "$D"
export OUT_SUBDIR=dwb_r4b
declare -A SC=(
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S3_ruido_yaw]='{"sigma_yaw":0.05,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
  [S6_combinado_lat0p6]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.6,"k_v":0.95,"k_w":0.92}'
)
for S in S1_nominal S2_ruido_xy S3_ruido_yaw S4_latencia_0p6 S5_combinado_medido S6_combinado_lat0p6; do
  /usr/bin/python3 "$WS/scripts/nav_offline_lab/make_r4b_runs.py" --paths "$WS/results/navigation_offline_lab/planner_runs/paths_R3.json" \
    --scenario "$S" --out "$D/runs_$S.txt" >/dev/null
  for V in A B; do
    CFG=base; [ $V = B ] && CFG=fp
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; print(json.dumps(d))")
    SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$WS/config/offline_lab/dwb_offline_R4B_$CFG.yaml" \
      "${S}_$V" "$D/runs_$S.txt" 2>&1 | grep -E "terminado|FALL|abort|timeout" | sed "s/^/$S $V: /"
    sleep 5
  done
done
echo TODO_TERMINADO
