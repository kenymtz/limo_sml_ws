#!/usr/bin/python3
"""NAV-OFFLINE-R4-A — genera casos de evaluación DWB a partir de paths NavFn ya calculados (R3).

Para cada path SUCCEEDED:
  <T>_S            pose = start del benchmark (yaw del benchmark), v=(0,0)            [detalle, 5 repeticiones]
  <T>_A<k>         path recortado desde s-BEHIND (emula poda en régimen); muestras cada STEP m de arco (desde STEP hasta L-END_MARGIN), yaw = tangente del path,
                   v=(V_CRUISE,0)  (instantánea estática, NO simulación en lazo cerrado)
  <T>_A<k>_{L+,L-,Y+,Y-}  perturbación de localización: lateral ±LAT m, yaw ±YAW rad (solo con --perturb)
Salida: formato texto para dwb_offline_eval (CASE/PATH) + índice JSON.
"""
import argparse
import json
import math

STEP, END_MARGIN, V_CRUISE, LAT, YAW = 0.5, 0.3, 0.25, 0.05, 0.15
# Emulación de la poda de DWB en régimen (prune_plan=true): en lazo cerrado global_plan_ se recorta cada ciclo y
# el BT replanifica desde la pose actual, así que en una muestra a s metros el plan empieza ~BEHIND m detrás del
# robot. Entregar el path completo desde el start con s > prune_distance (2.0 m) produce en Humble
# 'Resulting plan has 0 poses in it.' (artefacto del banco, medido en eval_INVALID_fullpath_nopruning.jsonl).
BEHIND = 1.4


def tangent(path, i, look=0.10):
    x0, y0 = path[i][0], path[i][1]
    for j in range(i + 1, len(path)):
        if math.hypot(path[j][0] - x0, path[j][1] - y0) >= look:
            return math.atan2(path[j][1] - y0, path[j][0] - x0)
    j = max(0, i - 1)
    return math.atan2(y0 - path[j][1], x0 - path[j][0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--paths', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--index', required=True)
    ap.add_argument('--perturb', action='store_true')
    ap.add_argument('--no-along', action='store_true')
    a = ap.parse_args()
    d = json.load(open(a.paths))
    lines, index = [], []

    def add(cid, test, kind, x, y, yaw, vx, wz, detail, reps, path, extra=None):
        lines.append(f'CASE {cid} {x:.6f} {y:.6f} {yaw:.6f} {vx:.4f} {wz:.4f} {detail} {reps}')
        lines.append(f'PATH {len(path)} ' + ' '.join(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}' for p in path))
        index.append(dict(case=cid, test=test, kind=kind, category=r['category'], **(extra or {})))

    for r in d['results']:
        if r['status'] != 'SUCCEEDED' or not r['path']:
            continue
        T, P = r['test_id'], r['path']
        s = [0.0]
        for i in range(1, len(P)):
            s.append(s[-1] + math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]))
        L = s[-1]
        sx, sy, syaw = r['start']
        yaw_path0 = tangent(P, 0)
        add(f'{T}_S', T, 'START', sx, sy, syaw, 0.0, 0.0, 1, 5, P,
            dict(path_len_m=L, start_heading_err_rad=math.atan2(math.sin(syaw - yaw_path0),
                                                                 math.cos(syaw - yaw_path0))))
        if a.no_along:
            continue
        k, target = 0, STEP
        for i in range(len(P)):
            if s[i] >= target and s[i] <= L - END_MARGIN:
                k += 1
                th = tangent(P, i)
                x, y = P[i][0], P[i][1]
                base = f'{T}_A{k:02d}'
                j0 = next(j for j in range(len(P)) if s[j] >= s[i] - BEHIND)
                ex = dict(s_m=s[i], path_len_m=L, path_trim_from_s=s[j0])
                P_full, P = P, P[j0:]
                add(base, T, 'ALONG', x, y, th, V_CRUISE, 0.0, 0, 1, P, ex)
                if a.perturb:
                    nx, ny = -math.sin(th), math.cos(th)
                    add(base + '_L+', T, 'PERT_LAT', x + LAT * nx, y + LAT * ny, th, V_CRUISE, 0.0, 0, 1, P, ex)
                    add(base + '_L-', T, 'PERT_LAT', x - LAT * nx, y - LAT * ny, th, V_CRUISE, 0.0, 0, 1, P, ex)
                    add(base + '_Y+', T, 'PERT_YAW', x, y, th + YAW, V_CRUISE, 0.0, 0, 1, P, ex)
                    add(base + '_Y-', T, 'PERT_YAW', x, y, th - YAW, V_CRUISE, 0.0, 0, 1, P, ex)
                P = P_full
                target += STEP
    open(a.out, 'w').write('\n'.join(lines) + '\n')
    json.dump(dict(source=a.paths, params=dict(BEHIND=BEHIND, STEP=STEP, END_MARGIN=END_MARGIN, V_CRUISE=V_CRUISE, LAT=LAT,
                                                YAW=YAW), cases=index), open(a.index, 'w'), indent=0)
    from collections import Counter
    print(len(index), Counter(c['kind'] for c in index))


if __name__ == '__main__':
    main()
