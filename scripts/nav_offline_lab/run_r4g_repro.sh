#!/bin/bash
# NAV-OFFLINE-R4-G Fase 1 — reproducción de los ABORT_PROGRESS cerca del goal de R4-F (yaw 0.10).
# Config: dwb_offline_R4F_010_sim.yaml (candidata R4-F, sin cambios). Mismos runs/semillas (líneas RUN/PATH copiadas de
# dwb_r4b/runs_<S>.txt) y mismos SIM_PARAMS que R4-D/E/F. Cada caso dos veces: debug OFF y debug ON (debug_eval_runs).
# Offline: run_dwb_r4a.sh fuerza ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99 y audita el grafo (sin Twist).
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4g"; mkdir -p "$D/repro_runs"
export OUT_SUBDIR=dwb_r4g
CFG="$WS/config/offline_lab/dwb_offline_R4F_010_sim.yaml"
declare -A SC=(
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
)
CASES=${R4G_CASES:-"S1_nominal:T041 S2_ruido_xy:T045 S5_combinado_medido:T041"}
for C in $CASES; do
  S=${C%%:*}; ID=${C##*:}
  RF="$D/repro_runs/runs_${S}_${ID}.txt"
  awk -v id="$ID" '$1=="RUN"{keep=($2==id)} keep' "$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt" > "$RF"
  [ "$(grep -c '^RUN' "$RF")" = 1 ] || { echo "caso $C no encontrado"; exit 2; }
  for MODE in nodbg dbg; do
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "
import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'
if '$MODE'=='dbg': d['debug_eval_runs']='$ID'
print(json.dumps(d))")
    SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$CFG" "repro_${S}_${ID}_${MODE}" "$RF" 2>&1 \
      | grep -E "terminado|FALL|abort|timeout|Abort" | sed "s/^/$C $MODE: /"
    sleep 3
  done
done
echo TODO_TERMINADO
