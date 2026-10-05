#!/bin/bash
# NAV-OFFLINE-R4-E — FollowPath.xy_goal_tolerance ∈ {0.20, 0.15, 0.10} (RotateToGoal + ObstacleFootprint), 6 escenarios R4-B.
# Baseline 0.25 = R4-D existente (NO se re-ejecuta). Mismos runs/semillas (dwb_r4b/runs_*.txt) y mismos SIM_PARAMS que R4-D.
# Offline: run_dwb_r4a.sh fuerza ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99 y audita el grafo (sin Twist) en cada corrida.
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4e"; mkdir -p "$D"
export OUT_SUBDIR=dwb_r4e
/usr/bin/python3 "$WS/scripts/nav_offline_lab/verify_r4e_configs.py" || { echo "verificación de configs FALLÓ"; exit 2; }
declare -A SC=(   # idéntico a run_r4d_all.sh
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S3_ruido_yaw]='{"sigma_yaw":0.05,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
  [S6_combinado_lat0p6]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.6,"k_v":0.95,"k_w":0.92}'
)
for TAG in ${R4E_TAGS:-020 015 010}; do
  for S in S1_nominal S2_ruido_xy S3_ruido_yaw S4_latencia_0p6 S5_combinado_medido S6_combinado_lat0p6; do
    RUNS="$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt"
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; print(json.dumps(d))")
    SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$WS/config/offline_lab/dwb_offline_R4E_${TAG}_sim.yaml" \
      "${S}_${TAG}" "$RUNS" 2>&1 | grep -E "terminado|FALL|abort|timeout|Abort" | sed "s/^/$TAG $S: /"
    sleep 5
  done
done
echo TODO_TERMINADO
