#!/usr/bin/python3
"""NAV-OFFLINE-R4-B — archivo de corridas para dwb_closed_loop_sim desde los 64 paths NavFn de R3 (mismos de R4-A/A2).
RUN <id> start(x y yaw) goal(x y yaw) <seed> + PATH. La semilla depende solo del test y del escenario, así que
A (BaseObstacle) y B (ObstacleFootprint) reciben EXACTAMENTE la misma secuencia de ruido (comparación pareada)."""
import argparse
import json
import zlib

ap = argparse.ArgumentParser()
ap.add_argument('--paths', required=True)
ap.add_argument('--scenario', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--only', default='')
a = ap.parse_args()
d = json.load(open(a.paths))
only = set(a.only.split(',')) if a.only else None
L = []
for r in d['results']:
    if r['status'] != 'SUCCEEDED' or (only and r['test_id'] not in only):
        continue
    seed = zlib.crc32(f"{r['test_id']}|{a.scenario}".encode())
    sx, sy, syaw = r['start']
    gx, gy, gyaw = r['goal']
    L.append(f"RUN {r['test_id']} {sx} {sy} {syaw} {gx} {gy} {gyaw} {seed}")
    L.append(f"PATH {len(r['path'])} " + ' '.join(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}' for p in r['path']))
open(a.out, 'w').write('\n'.join(L) + '\n')
print(len(L) // 2, 'runs')
