#!/usr/bin/python3
"""Validador offline de mapas ROS (formato nav2_map_server). Solo lectura.

Uso:
  validate_ros_map.py --map-yaml mapa.yaml [--scan-dir DIR ...] [--strict]
                      [--expect-resolution 0.03] [--max-unknown-pct 90] [--json]

Clasificación = réplica de nav2_map_server (Humble, map_io.cpp):
  occ = (255 - px)/255  (negate=0)  |  occ = px/255 (negate=1)
  trinary: occ > occupied_thresh -> OCCUPIED ; occ < free_thresh -> FREE ; resto UNKNOWN
  (validado contra el map_server real en NAV-OFFLINE-R1/R2)

Exit codes:
  0 PASS (puede haber WARN)   1 WARN con --strict   2 FAIL (error crítico)
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import yaml

SAVER_FREE, SAVER_UNKNOWN, SAVER_OCC = 254, 205, 0
FRAGILE_MARGIN = 1e-3


def read_image(path):
    """Devuelve array uint8 (H, W) en escala de grises. Soporta PGM P5/P2; otros vía cv2."""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:2] in (b'P5', b'P2'):
        tokens, idx = [], 0
        while len(tokens) < 4:
            while idx < len(data) and data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b'#':
                idx = data.index(b'\n', idx) + 1
                continue
            start = idx
            while idx < len(data) and not data[idx:idx + 1].isspace():
                idx += 1
            tokens.append(data[start:idx])
        w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
        if tokens[0] == b'P5':
            if maxval > 255:
                raise ValueError(f'PGM de 16 bits (maxval={maxval}) no soportado por este validador')
            payload = data[idx + 1:]
            if len(payload) < w * h:
                raise ValueError(f'PGM truncado: {len(payload)} B < {w * h}')
            arr = np.frombuffer(payload[:w * h], dtype=np.uint8).reshape(h, w)
            extra = len(payload) - w * h
        else:
            vals = np.array(data[idx:].split(), dtype=np.int64)
            arr = vals[:w * h].reshape(h, w).astype(np.uint8)
            extra = 0
        if maxval != 255:
            arr = np.round(arr.astype(float) * 255.0 / maxval).astype(np.uint8)
        return arr, {'format': tokens[0].decode(), 'maxval': maxval, 'trailing_bytes': extra}
    try:
        import cv2
    except ImportError as e:
        raise ValueError(f'formato no PGM y cv2 no disponible: {e}')
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError('cv2 no pudo leer la imagen')
    if img.ndim == 3:
        img = img[..., :3].mean(axis=2).round().astype(np.uint8)
    return img.astype(np.uint8), {'format': os.path.splitext(path)[1], 'maxval': 255, 'trailing_bytes': 0}


def classify_levels(levels, mode, negate, occ_t, free_t):
    """Clasificación Nav2 por nivel de píxel: dict nivel -> FREE/OCCUPIED/UNKNOWN/INTERMEDIATE."""
    out = {}
    for px in levels:
        px = int(px)
        if mode == 'raw':
            out[px] = 'FREE' if px == 0 else ('OCCUPIED' if px == 100 else
                                              ('UNKNOWN' if px > 100 else 'INTERMEDIATE'))
            continue
        shade = px / 255.0
        occ = shade if negate else 1.0 - shade
        if occ > occ_t:
            out[px] = 'OCCUPIED'
        elif occ < free_t:
            out[px] = 'FREE'
        else:
            out[px] = 'UNKNOWN' if mode == 'trinary' else 'INTERMEDIATE'
    return out


def resolve_image(yaml_path, image):
    return os.path.normpath(image if os.path.isabs(image) else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image))


def find_shared_image_conflicts(yaml_path, img_path, resolution, scan_dirs):
    """Busca otros yaml (en el dir del yaml + scan_dirs) que usen la misma imagen."""
    dirs = {os.path.dirname(os.path.abspath(yaml_path))} | {os.path.abspath(d) for d in scan_dirs}
    shared = []
    for d in sorted(dirs):
        for y in sorted(glob.glob(os.path.join(d, '*.yaml')) + glob.glob(os.path.join(d, '.*.yaml'))):
            if os.path.abspath(y) == os.path.abspath(yaml_path):
                continue
            try:
                m = yaml.safe_load(open(y))
            except Exception:
                continue
            if not isinstance(m, dict) or 'image' not in m:
                continue
            if resolve_image(y, str(m['image'])) == img_path:
                shared.append({'yaml': y, 'resolution': m.get('resolution'),
                               'different_resolution': m.get('resolution') != resolution})
    return shared


def validate(yaml_path, scan_dirs=(), expect_resolution=None, max_unknown_pct=90.0):
    r = {'yaml': os.path.abspath(yaml_path), 'critical': [], 'warnings': [], 'info': []}
    C, W, I = r['critical'].append, r['warnings'].append, r['info'].append

    if not os.path.isfile(yaml_path):
        C(f'YAML no existe: {yaml_path}')
        return finish(r)
    try:
        meta = yaml.safe_load(open(yaml_path))
    except Exception as e:
        C(f'YAML ilegible: {e}')
        return finish(r)
    if not isinstance(meta, dict):
        C('YAML no es un diccionario')
        return finish(r)
    r['yaml_fields'] = meta

    for k in ('image', 'resolution', 'origin'):
        if k not in meta:
            C(f'falta campo obligatorio "{k}"')
    mode = str(meta.get('mode', 'trinary'))
    if mode not in ('trinary', 'scale', 'raw'):
        C(f'mode inválido: {mode}')
    negate = meta.get('negate', 0)
    if negate not in (0, 1, True, False):
        C(f'negate inválido: {negate}')
    occ_t = meta.get('occupied_thresh', 0.65)
    free_t = meta.get('free_thresh', 0.25)
    for name, v in (('occupied_thresh', occ_t), ('free_thresh', free_t)):
        if not isinstance(v, (int, float)) or not (0.0 <= v <= 1.0):
            C(f'{name} fuera de [0,1] o no numérico: {v}')
    if isinstance(occ_t, (int, float)) and isinstance(free_t, (int, float)) and free_t >= occ_t:
        C(f'free_thresh ({free_t}) >= occupied_thresh ({occ_t})')

    res = meta.get('resolution')
    if 'resolution' in meta and (not isinstance(res, (int, float)) or not math.isfinite(res) or res <= 0):
        C(f'resolución inválida: {res}')
        res = None
    if res and expect_resolution is not None and abs(res - expect_resolution) > 1e-9:
        C(f'resolución {res} != esperada {expect_resolution}')
    origin = meta.get('origin')
    if 'origin' in meta:
        if (not isinstance(origin, (list, tuple)) or len(origin) != 3 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in origin)):
            C(f'origin inválido (se espera [x, y, yaw]): {origin}')
        elif origin[2] != 0:
            W(f'origin yaw = {origin[2]} != 0 (verificar soporte en el consumidor)')

    if r['critical'] or 'image' not in meta:
        return finish(r)

    img_path = resolve_image(yaml_path, str(meta['image']))
    r['image_path'] = img_path
    if not os.path.isfile(img_path):
        C(f'imagen no existe: {img_path}')
        return finish(r)

    # ¿la imagen corresponde al yaml?
    stem = os.path.splitext(os.path.basename(yaml_path))[0]
    img_stem = os.path.splitext(os.path.basename(img_path))[0]
    if img_stem != stem:
        siblings = [p for p in glob.glob(os.path.join(os.path.dirname(os.path.abspath(yaml_path)), stem + '.*'))
                    if os.path.splitext(p)[1].lower() in ('.pgm', '.png', '.bmp', '.jpg', '.jpeg')]
        if siblings and os.path.normpath(siblings[0]) != img_path:
            C(f'imagen no corresponde: el yaml "{stem}" apunta a "{os.path.basename(img_path)}" '
              f'pero existe su homónima "{os.path.basename(siblings[0])}"')
        else:
            I(f'nombre de imagen ({img_stem}) distinto del yaml ({stem}); sin homónima conflictiva')

    try:
        arr, fmt = read_image(img_path)
    except Exception as e:
        C(f'imagen ilegible: {e}')
        return finish(r)
    h, w = arr.shape
    r['image_format'] = fmt
    if fmt.get('trailing_bytes'):
        W(f'PGM con {fmt["trailing_bytes"]} bytes sobrantes tras el payload')
    if h == 0 or w == 0:
        C('imagen vacía')
        return finish(r)
    r['size_px'] = [w, h]
    if res:
        r['size_m'] = [round(w * res, 4), round(h * res, 4)]
        if origin and len(origin) == 3:
            r['extent_m'] = {'x': [origin[0], round(origin[0] + w * res, 4)],
                             'y': [origin[1], round(origin[1] + h * res, 4)]}

    levels, counts = np.unique(arr, return_counts=True)
    hist = {int(a): int(b) for a, b in zip(levels, counts)}
    r['pixel_hist'] = hist if len(hist) <= 20 else {'n_levels': len(hist)}
    cls = classify_levels(levels, mode, bool(negate), occ_t, free_t)
    n = arr.size
    tot = {'FREE': 0, 'OCCUPIED': 0, 'UNKNOWN': 0, 'INTERMEDIATE': 0}
    for px, c in hist.items():
        tot[cls[px]] += c
    r['nav2_counts'] = tot
    r['nav2_pct'] = {k: round(100 * v / n, 3) for k, v in tot.items()}
    r['level_classification'] = {str(k): v for k, v in cls.items()} if len(cls) <= 20 else None

    if mode == 'trinary':
        # peligro principal: desconocido guardado (205) leído como libre
        if SAVER_UNKNOWN in hist and cls[SAVER_UNKNOWN] == 'FREE':
            C(f'{hist[SAVER_UNKNOWN]} píxeles 205 (UNKNOWN de map_saver) se cargan como FREE '
              f'con free_thresh={free_t} (usar free_thresh <= 0.196; ver MAP_THRESHOLD_PROVENANCE.md)')
        if SAVER_FREE in hist and cls[SAVER_FREE] != 'FREE':
            C(f'píxeles 254 (FREE de map_saver) se cargan como {cls[SAVER_FREE]}')
        if SAVER_OCC in hist and cls[SAVER_OCC] != 'OCCUPIED':
            C(f'píxeles 0 (OCCUPIED de map_saver) se cargan como {cls[SAVER_OCC]}')
        occ205 = (255 - SAVER_UNKNOWN) / 255.0
        if not negate and free_t <= occ205 and occ205 - free_t < FRAGILE_MARGIN:
            W(f'free_thresh={free_t} deja 205 como UNKNOWN con margen {occ205 - free_t:.1e} (frágil)')
        others = sorted(set(hist) - {SAVER_FREE, SAVER_UNKNOWN, SAVER_OCC})
        if others:
            W(f'{len(others)} niveles de gris fuera de {{0,205,254}} '
              f'({sum(hist[o] for o in others)} px); ej: ' +
              ', '.join(f'{o}->{cls[o]}' for o in others[:8]))
    if tot['FREE'] == 0:
        W('el mapa no tiene celdas FREE')
    if tot['OCCUPIED'] == 0:
        W('el mapa no tiene celdas OCCUPIED')
    if r['nav2_pct']['UNKNOWN'] > max_unknown_pct:
        W(f'UNKNOWN {r["nav2_pct"]["UNKNOWN"]:.1f}% > {max_unknown_pct}%')

    shared = find_shared_image_conflicts(yaml_path, img_path, res, scan_dirs)
    r['shared_image_yamls'] = shared
    for s in shared:
        if s['different_resolution']:
            W(f'la imagen también la usa {s["yaml"]} con resolución {s["resolution"]} (≠ {res})')
        else:
            I(f'la imagen también la usa {s["yaml"]} (misma resolución)')
    return finish(r)


def finish(r):
    r['status'] = 'FAIL' if r['critical'] else ('WARN' if r['warnings'] else 'PASS')
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map-yaml', required=True)
    ap.add_argument('--scan-dir', action='append', default=[],
                    help='directorios adicionales donde buscar yaml que compartan la imagen')
    ap.add_argument('--expect-resolution', type=float)
    ap.add_argument('--max-unknown-pct', type=float, default=90.0)
    ap.add_argument('--strict', action='store_true', help='WARN -> exit 1')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    r = validate(a.map_yaml, a.scan_dir, a.expect_resolution, a.max_unknown_pct)
    if a.json:
        print(json.dumps(r, indent=1, default=str))
    else:
        print(f'[{r["status"]}] {r["yaml"]}')
        if 'size_px' in r:
            print(f'  imagen: {r.get("image_path")}  {r["size_px"][0]}x{r["size_px"][1]} px'
                  + (f'  = {r["size_m"][0]} x {r["size_m"][1]} m' if 'size_m' in r else ''))
            f = r['yaml_fields']
            print(f'  res={f.get("resolution")} origin={f.get("origin")} mode={f.get("mode", "trinary")} '
                  f'negate={f.get("negate", 0)} occ={f.get("occupied_thresh")} free={f.get("free_thresh")}')
            print(f'  píxeles: {r["pixel_hist"]}')
            print(f'  Nav2: FREE {r["nav2_pct"]["FREE"]}%  OCC {r["nav2_pct"]["OCCUPIED"]}%  '
                  f'UNKNOWN {r["nav2_pct"]["UNKNOWN"]}%' +
                  (f'  INTERMEDIATE {r["nav2_pct"]["INTERMEDIATE"]}%' if r['nav2_counts']['INTERMEDIATE'] else ''))
        for k, tag in (('critical', 'CRITICAL'), ('warnings', 'WARN'), ('info', 'INFO')):
            for m in r[k]:
                print(f'  {tag}: {m}')
    if r['status'] == 'FAIL':
        return 2
    if r['status'] == 'WARN' and a.strict:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
