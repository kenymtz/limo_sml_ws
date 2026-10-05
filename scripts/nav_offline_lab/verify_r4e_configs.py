#!/usr/bin/python3
"""NAV-OFFLINE-R4-E — verificador automático: entre R4-D y cada variante R4-E SOLO cambia FollowPath.xy_goal_tolerance.

Comprueba (exit 1 si algo falla):
  1. configs (completa y simulador): diff de dicts aplanados vs R4-D == {controller_server...FollowPath.xy_goal_tolerance}
  2. general_goal_checker.xy/yaw_goal_tolerance == 0.25 / 0.25 en todas
  3. producción config/navigation/navigation2_real.yaml con sha256 == el registrado en R4-D
  4. --runs: cada sim_<S>_<TAG>.jsonl de R4-E tiene meta idéntica a la de R4-D (escenario, ruido, latencia, límites,
     checker) y los mismos runs (id, start, goal, seed, path) en el mismo orden que dwb_r4b/runs_<S>.txt
Escribe results/navigation_offline_lab/dwb_r4e/config_verification.json
"""
import argparse
import hashlib
import json
import os
import sys

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
RES = os.path.join(WS, 'results/navigation_offline_lab')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
KEY = 'controller_server.ros__parameters.FollowPath.xy_goal_tolerance'
TAGS = {'020': 0.20, '015': 0.15, '010': 0.10}


def flat(d, p=''):
    out = {}
    for k, v in d.items():
        kk = f'{p}.{k}' if p else str(k)
        if isinstance(v, dict):
            out.update(flat(v, kk))
        else:
            out[kk] = v
    return out


def diff(a, b):
    fa, fb = flat(a), flat(b)
    return sorted(k for k in set(fa) | set(fb) if fa.get(k, '<absent>') != fb.get(k, '<absent>'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', action='store_true', help='verificar también las salidas sim_*.jsonl de R4-E')
    a = ap.parse_args()
    rep, ok = dict(checks=[]), True

    def check(name, cond, detail=None):
        nonlocal ok
        ok &= bool(cond)
        rep['checks'].append(dict(check=name, PASS=bool(cond), detail=detail))
        print(('PASS ' if cond else 'FAIL ') + name + (f'  {detail}' if detail is not None and not cond else ''))

    real_sha = hashlib.sha256(open(REAL, 'rb').read()).hexdigest()
    r4d_man = json.load(open(os.path.join(LAB, 'dwb_offline_R4D_manifest.json')))
    check('produccion navigation2_real.yaml sin cambios', real_sha == r4d_man['real_sha256'], real_sha)
    base = {'full': yaml.safe_load(open(os.path.join(LAB, 'navigation2_R4D.yaml'))),
            'sim': yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4D_fp_rtg.yaml')))}
    for kind in base:
        check(f'R4-D {kind}: FollowPath.xy_goal_tolerance == 0.25', flat(base[kind])[KEY] == 0.25)
    for tag, tol in TAGS.items():
        for kind, name in (('full', f'navigation2_R4E_{tag}.yaml'), ('sim', f'dwb_offline_R4E_{tag}_sim.yaml')):
            d = yaml.safe_load(open(os.path.join(LAB, name)))
            dd = diff(base[kind], d)
            check(f'{name}: diff vs R4-D == [{KEY}]', dd == [KEY], dd)
            check(f'{name}: {KEY} == {tol}', flat(d)[KEY] == tol, flat(d)[KEY])
            gc = d['controller_server']['ros__parameters']['general_goal_checker']
            check(f'{name}: general_goal_checker 0.25/0.25',
                  gc['xy_goal_tolerance'] == 0.25 and gc['yaw_goal_tolerance'] == 0.25, gc)
        # completa vs simulador: mismo controller_server
        full = yaml.safe_load(open(os.path.join(LAB, f'navigation2_R4E_{tag}.yaml')))
        sim = yaml.safe_load(open(os.path.join(LAB, f'dwb_offline_R4E_{tag}_sim.yaml')))
        check(f'R4E_{tag}: controller_server(simulador) == controller_server(candidata completa)',
              full['controller_server'] == sim['controller_server'])
    if a.runs:
        for s in SCEN:
            with open(os.path.join(RES, 'dwb_r4d', f'sim_{s}_R.jsonl')) as f:
                meta_d = json.loads(f.readline())['meta']
            runs_ref = [ln.split()[:8] for ln in open(os.path.join(RES, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')]
            for tag in TAGS:
                fn = os.path.join(RES, 'dwb_r4e', f'sim_{s}_{tag}.jsonl')
                if not os.path.exists(fn):
                    check(f'{s}/{tag}: salida existe', False, fn)
                    continue
                L = [json.loads(z) for z in open(fn)]
                check(f'{s}/{tag}: meta idéntica a R4-D', L[0]['meta'] == meta_d,
                      {k: (meta_d.get(k), L[0]['meta'].get(k)) for k in set(meta_d) | set(L[0]['meta'])
                       if meta_d.get(k) != L[0]['meta'].get(k)})
                ids = [x['run'] for x in L[1:]]
                check(f'{s}/{tag}: mismos 64 runs y orden', ids == [r[1] for r in runs_ref], len(ids))
                goals_ok = all(abs(x['goal'][i] - float(r[5 + i])) < 1e-9 for x, r in zip(L[1:], runs_ref) for i in range(3))
                check(f'{s}/{tag}: mismos goals', goals_ok)
    rep['PASS'] = bool(ok)
    os.makedirs(os.path.join(RES, 'dwb_r4e'), exist_ok=True)
    json.dump(rep, open(os.path.join(RES, 'dwb_r4e', 'config_verification.json' if not a.runs else 'run_verification.json'), 'w'),
              indent=1, default=str)
    print('TODO PASS' if ok else 'HAY FALLAS')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
