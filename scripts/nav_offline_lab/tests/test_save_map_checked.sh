#!/bin/bash
# Test OFFLINE de save_map_checked.sh (dominio 99, localhost, sin hardware).
# Salidas en results/navigation_offline_lab/save_pipeline_test/<timestamp>/
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
S="$HERE/../save_map_checked.sh"
BASE="$HOME/limo_sml_ws/results/navigation_offline_lab/save_pipeline_test/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$BASE")"
PASS=0; FAIL=0
check() {  # nombre, exit esperado, exit obtenido
  if [ "$2" = "$3" ]; then echo "PASS  $1 (exit $3)"; PASS=$((PASS+1)); else echo "FAIL  $1 (esperado $2, obtenido $3)"; FAIL=$((FAIL+1)); fi
}

/usr/bin/python3 "$HERE/fake_map_publisher.py" --topic /offline_test/map --duration 150 >/dev/null 2>&1 &
PUB=$!
trap 'kill $PUB $PUB2 2>/dev/null; wait 2>/dev/null' EXIT
sleep 2

"$S" --name synth_nominal --topic /offline_test/map --base-dir "$BASE" --expect-resolution 0.03 --verify-map-server > "$BASE.nominal.log" 2>&1
check "nominal (guardado+validación+snapshot+map_server)" 0 $?
"$S" --name synth_nominal --topic /offline_test/map --base-dir "$BASE" > /dev/null 2>&1
check "destino existente no se sobrescribe" 2 $?
"$S" --name map --topic /offline_test/map --base-dir "$BASE" > /dev/null 2>&1
check "nombre genérico 'map' rechazado" 2 $?
"$S" --name synth_x --topic /offline_test/map --base-dir "$HOME/limoatwork/should_not_exist" > /dev/null 2>&1
check "destino en ~/limoatwork rechazado" 2 $?
[ ! -e "$HOME/limoatwork/should_not_exist" ]; check "no se creó nada en ~/limoatwork" 0 $?
"$S" --name synth_x --topic /offline_test/map --base-dir /tmp/fuera > /dev/null 2>&1
check "destino fuera de results/ rechazado" 2 $?
"$S" --name synth_badfree --topic /offline_test/map --base-dir "$BASE" --load-free 0.25 > /dev/null 2>&1
check "load_free 0.25 rechazado" 2 $?
"$S" --name synth_badres --topic /offline_test/map --base-dir "$BASE" --expect-resolution 0.05 > /dev/null 2>&1
check "resolución inesperada -> FAIL" 2 $?
"$S" --name synth_notopic --topic /offline_test/no_existe --base-dir "$BASE" > /dev/null 2>&1
check "topic sin mapa -> FAIL" 2 $?

kill $PUB 2>/dev/null; wait $PUB 2>/dev/null
/usr/bin/python3 "$HERE/fake_map_publisher.py" --topic /offline_test/map_changing --changing --duration 90 >/dev/null 2>&1 &
PUB2=$!
sleep 2
"$S" --name synth_changing --topic /offline_test/map_changing --base-dir "$BASE" > "$BASE.changing.log" 2>&1
check "mapa cambiando durante el guardado -> FAIL" 2 $?

echo "RESUMEN: $PASS PASS, $FAIL FAIL   (salidas: $BASE)"
[ $FAIL -eq 0 ]
