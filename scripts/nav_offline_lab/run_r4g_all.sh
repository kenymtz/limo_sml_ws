#!/bin/bash
# NAV-OFFLINE-R4-G Fase 3 — progress checker (A: required_movement_radius 0.05/0.03; B: movement_time_allowance 35/45) sobre R4-F 0.10.
# Baseline = R4-F yaw 0.10 existente (NO se re-ejecuta). Mismos runs/semillas (dwb_r4b/runs_*.txt) y mismos SIM_PARAMS que R4-D.
# Offline: run_dwb_r4a.sh fuerza ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99 y audita el grafo (sin Twist) en cada corrida.
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4g"; mkdir -p "$D"
export OUT_SUBDIR=dwb_r4g
/usr/bin/python3 "$WS/scripts/nav_offline_lab/verify_r4g_configs.py" || { echo "verificación de configs FALLÓ"; exit 2; }
declare -A SC=(   # idéntico a run_r4d_all.sh
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S3_ruido_yaw]='{"sigma_yaw":0.05,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
  [S6_combinado_lat0p6]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.6,"k_v":0.95,"k_w":0.92}'
)
for TAG in ${R4G_TAGS:-PR005 PR003 PT35 PT45}; do
  for S in S1_nominal S2_ruido_xy S3_ruido_yaw S4_latencia_0p6 S5_combinado_medido S6_combinado_lat0p6; do
    RUNS="$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt"
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; print(json.dumps(d))")
    SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$WS/config/offline_lab/dwb_offline_R4G_${TAG}_sim.yaml" \
      "${S}_${TAG}" "$RUNS" 2>&1 | grep -E "terminado|FALL|abort|timeout|Abort" | sed "s/^/$TAG $S: /"
    sleep 5
  done
done
echo TODO_TERMINADO
