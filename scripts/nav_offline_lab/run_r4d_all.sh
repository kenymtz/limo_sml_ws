#!/bin/bash
# NAV-OFFLINE-R4-D — variante EXPERIMENTAL (ObstacleFootprint + RotateToGoal) en los 6 escenarios de R4-B.
# Baseline = variante B de R4-B (reproducida idéntica con el binario actual: sim_chk_base_*). Mismos runs/semillas.
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4d"; mkdir -p "$D"
export OUT_SUBDIR=dwb_r4d
declare -A SC=(
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S3_ruido_yaw]='{"sigma_yaw":0.05,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
  [S6_combinado_lat0p6]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.6,"k_v":0.95,"k_w":0.92}'
)
for S in S1_nominal S2_ruido_xy S3_ruido_yaw S4_latencia_0p6 S5_combinado_medido S6_combinado_lat0p6; do
  RUNS="$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt"   # mismos archivos (mismas semillas) que R4-B
  P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; print(json.dumps(d))")
  SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$WS/config/offline_lab/dwb_offline_R4D_fp_rtg.yaml" \
    "${S}_R" "$RUNS" 2>&1 | grep -E "terminado|FALL|abort|timeout" | sed "s/^/$S R: /"
  sleep 5
done
echo TODO_TERMINADO
