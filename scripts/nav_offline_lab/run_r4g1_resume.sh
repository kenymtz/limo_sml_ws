#!/bin/bash
# NAV-OFFLINE-R4-G1 (REANUDACIÓN: salta (TAG,S) con salida completa de 65 líneas) — baseline completa en plan_mode=replan_1hz (64 paths × 6 escenarios, mismas semillas que R4-F).
# Uso: run_r4g1.sh [TAG ...]   TAG ∈ F010 (candidata R4-F, por defecto) | E015 | D | B_B | B_A  (cadena de re-validación)
# Orden: (1) verificación estática  (2) autotest funcional (NavFn/costmap limpio/plan inicial/independencia)  (3) corridas.
# Offline: run_dwb_r4a.sh fuerza ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99 y audita el grafo (sin Twist, sin acciones).
set -u
WS="$HOME/limo_sml_ws"; D="$WS/results/navigation_offline_lab/dwb_r4g1"; mkdir -p "$D/selftest"
TAGS=("$@"); [ ${#TAGS[@]} -eq 0 ] && TAGS=(F010)
V="$WS/scripts/nav_offline_lab/verify_r4g1.py"
/usr/bin/python3 "$V" static > "$D/verify_static.log" 2>&1 || { echo "VERIFICACIÓN ESTÁTICA FALLÓ"; cat "$D/verify_static.log"; exit 2; }
declare -A SC=(   # copia literal de run_r4f_all.sh (verify_r4g1.py static lo compara)
  [S1_nominal]='{}'
  [S2_ruido_xy]='{"sigma_xy":0.03,"noise_tau":1.0}'
  [S3_ruido_yaw]='{"sigma_yaw":0.05,"noise_tau":1.0}'
  [S4_latencia_0p6]='{"latency":0.6}'
  [S5_combinado_medido]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.05,"k_v":0.95,"k_w":0.92}'
  [S6_combinado_lat0p6]='{"sigma_xy":0.03,"sigma_yaw":0.05,"noise_tau":1.0,"latency":0.6,"k_v":0.95,"k_w":0.92}'
)
if [ ! -f "$D/selftest/PASS" ]; then
  # autotest: S1, casos que recorren N1 seguidos (T039 deja rastro si el bug de clearRobotCell volviera) + T005
  RF="$D/selftest/runs_selftest.txt"
  /usr/bin/python3 - "$WS/results/navigation_offline_lab/dwb_r4b/runs_S1_nominal.txt" "$RF" T005 T039 T041 T043 T045 <<'EOF'
import sys
src, dst, ids = sys.argv[1], sys.argv[2], set(sys.argv[3:])
out, keep = [], False
for ln in open(src):
    if ln.startswith('RUN'):
        keep = ln.split()[1] in ids
    if keep:
        out.append(ln)
open(dst, 'w').writelines(out)
EOF
  OUT_SUBDIR=dwb_r4g1/selftest SIM_PARAMS='{"scenario":"S1_nominal","plan_mode":"replan_1hz"}' \
    "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$WS/config/offline_lab/dwb_offline_R4G1_F010_replan_sim.yaml" \
    selftest "$RF" 2>&1 | grep -E "terminado|FALL|abort" | sed "s/^/selftest: /"
  /usr/bin/python3 "$V" selftest > "$D/verify_selftest.log" 2>&1 || { echo "AUTOTEST FALLÓ"; cat "$D/verify_selftest.log"; exit 3; }
  touch "$D/selftest/PASS"
  sleep 3
fi
for TAG in "${TAGS[@]}"; do
  CFG="$WS/config/offline_lab/dwb_offline_R4G1_${TAG}_replan_sim.yaml"
  for S in S1_nominal S2_ruido_xy S3_ruido_yaw S4_latencia_0p6 S5_combinado_medido S6_combinado_lat0p6; do
    RUNS="$WS/results/navigation_offline_lab/dwb_r4b/runs_$S.txt"
    F="$D/sim_${S}_${TAG}.jsonl"
    if [ -f "$F" ] && [ "$(wc -l < "$F")" = 65 ]; then echo "$TAG $S: ya completo, se salta"; continue; fi
    P=$(echo "${SC[$S]}" | /usr/bin/python3 -c "import sys,json; d=json.loads(sys.stdin.read()); d['scenario']='$S'; d['plan_mode']='replan_1hz'; print(json.dumps(d))")
    OUT_SUBDIR=dwb_r4g1 SIM_PARAMS="$P" "$WS/scripts/nav_offline_lab/run_dwb_r4a.sh" closedloop "$CFG" "${S}_${TAG}" "$RUNS" 2>&1 \
      | grep -E "terminado|FALL|abort|timeout|Abort|vacío|valida" | sed "s/^/$TAG $S: /"
    sleep 5
  done
done
echo TODO_TERMINADO
