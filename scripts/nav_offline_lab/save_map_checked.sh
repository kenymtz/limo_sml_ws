#!/bin/bash
# save_map_checked.sh — guardado de mapa con verificación (NAV-OFFLINE-R2).
#
# NO mueve el robot: solo SUSCRIBE a un OccupancyGrid y ejecuta nav2 map_saver_cli.
# No publica /cmd_vel, no llama servicios de SLAM, no toca ~/limoatwork.
#
# Uso:
#   save_map_checked.sh --name arena_YYYYMMDD_HHMM [--topic /map] [--base-dir DIR]
#                       [--expect-resolution 0.03] [--load-free 0.10]
#                       [--save-free 0.25] [--save-occ 0.65] [--verify-map-server] [--allow-warn]
#
# Resultado en  <base-dir>/<name>/   (base-dir por defecto ~/limo_sml_ws/results/maps)
#   <name>.pgm                 imagen guardada por map_saver_cli (solo lectura)
#   <name>.saver_raw.yaml      yaml ORIGINAL del saver, sin tocar (solo lectura)
#   <name>.yaml                yaml de CARGA (idéntico salvo free_thresh = --load-free)
#   <name>.snap_before.npz / .snap_after.npz   instantáneas del OccupancyGrid
#   <name>.save_report.json, SHA256SUMS, save.log
# Exit: 0 OK · 2 condición peligrosa / error (el mapa NO debe usarse)
set -u
WS="$HOME/limo_sml_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=/usr/bin/python3
NAME=""; TOPIC=/map; BASE="$WS/results/maps"; EXPECT_RES=""; LOAD_FREE=0.10
SAVE_FREE=0.25; SAVE_OCC=0.65; VERIFY_MS=0; ALLOW_WARN=0

die() { echo "FAIL: $*" >&2; [ -n "${DEST:-}" ] && [ -d "${DEST:-}" ] && echo "FAIL: $*" >> "$DEST/save.log"; exit 2; }

while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift 2;;
    --topic) TOPIC="$2"; shift 2;;
    --base-dir) BASE="$2"; shift 2;;
    --expect-resolution) EXPECT_RES="$2"; shift 2;;
    --load-free) LOAD_FREE="$2"; shift 2;;
    --save-free) SAVE_FREE="$2"; shift 2;;
    --save-occ) SAVE_OCC="$2"; shift 2;;
    --verify-map-server) VERIFY_MS=1; shift;;
    --allow-warn) ALLOW_WARN=1; shift;;
    -h|--help) sed -n 2,24p "$0"; exit 0;;
    *) die "argumento desconocido: $1";;
  esac
done

# ---------- 0. validaciones de entrada ----------
[ -n "$NAME" ] || die "falta --name"
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]] || die "nombre inválido '$NAME' (A-Z a-z 0-9 _ -, 3-64 chars)"
case "$NAME" in map|mapa|maps|test|tmp) die "nombre genérico prohibido '$NAME' (causa sobrescrituras como map.pgm compartido)";; esac
# resolver SIN crear nada antes de comprobar (realpath -m no requiere que exista)
BASE_REAL="$(realpath -m "$BASE")"
case "$BASE_REAL/" in
  "$(realpath -m "$HOME/limoatwork")"/*) die "destino dentro de ~/limoatwork (INMUTABLE)";;
  "$(realpath -m "$WS/results")"/*) ;;
  *) die "destino fuera de ~/limo_sml_ws/results: $BASE_REAL";;
esac
mkdir -p "$BASE_REAL" || die "no se pudo crear $BASE_REAL"
DEST="$BASE_REAL/$NAME"
[ -e "$DEST" ] && die "destino ya existe (no se sobrescribe): $DEST"
$PY -c "import sys; f=float(sys.argv[1]); o=float(sys.argv[2]); l=float(sys.argv[3]); \
assert 0<f<o<1 and 0<l<=0.196, 'umbrales'" "$SAVE_FREE" "$SAVE_OCC" "$LOAD_FREE" 2>/dev/null \
  || die "umbrales inválidos: save_free=$SAVE_FREE save_occ=$SAVE_OCC load_free=$LOAD_FREE (load_free debe ser <=0.196)"

set +u; source /opt/ros/humble/setup.bash; set -u
mkdir -p "$DEST"
LOG="$DEST/save.log"
{
  echo "save_map_checked $(date -Is)"; echo "name=$NAME topic=$TOPIC dest=$DEST"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
  echo "save_free=$SAVE_FREE save_occ=$SAVE_OCC load_free=$LOAD_FREE expect_res=${EXPECT_RES:-none}"
} > "$LOG"

# ---------- 1. instantánea previa ----------
echo "[1/7] instantánea previa de $TOPIC"
$PY "$HERE/save_map_helper.py" snapshot --topic "$TOPIC" --out "$DEST/$NAME.snap_before.npz" >> "$LOG" 2>&1 \
  || die "sin OccupancyGrid en $TOPIC (¿SLAM/map_server activo en este ROS_DOMAIN_ID?)"

# ---------- 2. guardado ----------
echo "[2/7] map_saver_cli (umbrales de guardado explícitos)"
timeout 60 /opt/ros/humble/lib/nav2_map_server/map_saver_cli -t "$TOPIC" -f "$DEST/$NAME" \
  --occ "$SAVE_OCC" --free "$SAVE_FREE" --fmt pgm --mode trinary \
  --ros-args -r __node:=save_map_checked_saver >> "$LOG" 2>&1 || die "map_saver_cli falló (ver $LOG)"
[ -f "$DEST/$NAME.pgm" ] && [ -f "$DEST/$NAME.yaml" ] || die "map_saver_cli no generó pgm+yaml"

# ---------- 3. instantánea posterior ----------
echo "[3/7] instantánea posterior"
$PY "$HERE/save_map_helper.py" snapshot --topic "$TOPIC" --out "$DEST/$NAME.snap_after.npz" >> "$LOG" 2>&1 \
  || die "sin OccupancyGrid tras el guardado"

# ---------- 4. preservar yaml bruto y crear yaml de carga ----------
echo "[4/7] preservar yaml bruto + yaml de carga (free_thresh=$LOAD_FREE)"
mv "$DEST/$NAME.yaml" "$DEST/$NAME.saver_raw.yaml"
$PY "$HERE/save_map_helper.py" load-yaml --raw "$DEST/$NAME.saver_raw.yaml" --out "$DEST/$NAME.yaml" \
  --free "$LOAD_FREE" >> "$LOG" 2>&1 || die "no se pudo generar yaml de carga"

# ---------- 5. validación ----------
echo "[5/7] validate_ros_map.py"
VARGS=(--map-yaml "$DEST/$NAME.yaml" --json)
[ -n "$EXPECT_RES" ] && VARGS+=(--expect-resolution "$EXPECT_RES")
[ "$ALLOW_WARN" = 1 ] || VARGS+=(--strict)
$PY "$HERE/validate_ros_map.py" "${VARGS[@]}" > "$DEST/$NAME.validation.json"; VRC=$?
$PY "$HERE/validate_ros_map.py" --map-yaml "$DEST/$NAME.saver_raw.yaml" --json > "$DEST/$NAME.validation_raw.json"
[ $VRC -eq 0 ] || die "validación del yaml de carga: exit $VRC (ver $NAME.validation.json)"

# ---------- 6. PGM vs instantáneas (UNKNOWN sigue UNKNOWN, res/origin/dims) ----------
echo "[6/7] PGM vs instantáneas"
$PY "$HERE/save_map_helper.py" check --pgm "$DEST/$NAME.pgm" --yaml "$DEST/$NAME.yaml" \
  --snap "$DEST/$NAME.snap_before.npz" --snap "$DEST/$NAME.snap_after.npz" \
  --save-free "$SAVE_FREE" --save-occ "$SAVE_OCC" > "$DEST/$NAME.snapshot_check.json" \
  || die "el PGM no corresponde a ninguna instantánea (ver $NAME.snapshot_check.json)"

# ---------- 7. (opcional) carga real en nav2_map_server aislado ----------
if [ "$VERIFY_MS" = 1 ]; then
  echo "[7/7] nav2_map_server aislado (dominio 99)"
  ( timeout 60 "$HERE/verify_map_server_load.sh" "$DEST/$NAME.yaml" > "$DEST/$NAME.map_server_check.json" ) \
    || die "verificación map_server falló"
  $PY - "$DEST/$NAME.map_server_check.json" "$DEST/$NAME.validation.json" <<'PY' || die "map_server real ≠ validador"
import json, sys
ms = json.load(open(sys.argv[1]))['map']; v = json.load(open(sys.argv[2]))['nav2_counts']
ok = (ms['free_0'], ms['occupied_100'], ms['unknown_-1']) == (v['FREE'], v['OCCUPIED'], v['UNKNOWN'])
print('map_server == validador' if ok else f'DIFERENCIA ms={ms} val={v}'); sys.exit(0 if ok else 1)
PY
else
  echo "[7/7] (omitido: --verify-map-server no indicado)"
fi

# ---------- informe + sellado ----------
$PY - "$DEST" "$NAME" <<'PY'
import json, os, sys
d, n = sys.argv[1], sys.argv[2]
v = json.load(open(f'{d}/{n}.validation.json')); vr = json.load(open(f'{d}/{n}.validation_raw.json'))
sc = json.load(open(f'{d}/{n}.snapshot_check.json'))
rep = {'name': n, 'dir': d, 'status': 'OK', 'load_yaml': f'{n}.yaml', 'raw_yaml': f'{n}.saver_raw.yaml',
       'load_validation': v['status'], 'load_nav2_pct': v.get('nav2_pct'),
       'raw_validation': vr['status'], 'raw_critical': vr['critical'],
       'snapshot_match': [s['match'] for s in sc['snapshots']], 'size_m': v.get('size_m'),
       'resolution': v['yaml_fields']['resolution'], 'origin': v['yaml_fields']['origin']}
json.dump(rep, open(f'{d}/{n}.save_report.json', 'w'), indent=1); print(json.dumps(rep, indent=1))
PY
( cd "$DEST" && sha256sum "$NAME.pgm" "$NAME.saver_raw.yaml" "$NAME.yaml" > SHA256SUMS )
chmod a-w "$DEST/$NAME.pgm" "$DEST/$NAME.saver_raw.yaml" "$DEST/$NAME.yaml" "$DEST/SHA256SUMS"
echo "OK: mapa guardado y verificado en $DEST  (usar $NAME.yaml para cargar)" | tee -a "$LOG"
exit 0
