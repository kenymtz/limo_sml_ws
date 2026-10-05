#!/usr/bin/python3
"""NAV-OFFLINE-R4-F — verificador: entre R4-E 0.15 y cada variante R4-F SOLO cambia general_goal_checker.yaw_goal_tolerance.

Comprueba (exit 1 si algo falla):
  1. configs (completa y simulador): diff de dicts aplanados vs R4-E 0.15 == {...general_goal_checker.yaw_goal_tolerance}
  2. general_goal_checker.xy_goal_tolerance == 0.25 y FollowPath.xy_goal_tolerance == 0.15 en todas
  3. controller_server(simulador) == controller_server(candidata completa)
  4. producción config/navigation/navigation2_real.yaml con el sha256 registrado en R4-D
  5. --runs: meta de cada dwb_r4f/sim_<S>_<tag>.jsonl idéntica a la de R4-E 0.15 salvo goal_yaw (== tolerancia de la variante),
     mismos 64 runs, orden y goals que dwb_r4b/runs_<S>.txt
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
CS = 'controller_server.ros__parameters.'
KEY = CS + 'general_goal_checker.yaw_goal_tolerance'
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
    ap.add_argument('--runs', action='store_true')
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
    base = {'full': yaml.safe_load(open(os.path.join(LAB, 'navigation2_R4E_015.yaml'))),
            'sim': yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4E_015_sim.yaml')))}
    for kind in base:
        check(f'R4-E 0.15 {kind}: yaw_goal_tolerance == 0.25', flat(base[kind])[KEY] == 0.25)
    for tag, yaw in TAGS.items():
        docs = {}
        for kind, name in (('full', f'navigation2_R4F_{tag}.yaml'), ('sim', f'dwb_offline_R4F_{tag}_sim.yaml')):
            d = docs[kind] = yaml.safe_load(open(os.path.join(LAB, name)))
            f = flat(d)
            dd = diff(base[kind], d)
            check(f'{name}: diff vs R4-E 0.15 == [{KEY}]', dd == [KEY], dd)
            check(f'{name}: yaw_goal_tolerance == {yaw}', f[KEY] == yaw, f[KEY])
            check(f'{name}: checker xy 0.25 y FollowPath xy 0.15',
                  f[CS + 'general_goal_checker.xy_goal_tolerance'] == 0.25 and f[CS + 'FollowPath.xy_goal_tolerance'] == 0.15)
        check(f'R4F_{tag}: controller_server(simulador) == controller_server(candidata completa)',
              docs['full']['controller_server'] == docs['sim']['controller_server'])
    if a.runs:
        for s in SCEN:
            with open(os.path.join(RES, 'dwb_r4e', f'sim_{s}_015.jsonl')) as f:
                meta_b = json.loads(f.readline())['meta']
            runs_ref = [ln.split()[:9] for ln in open(os.path.join(RES, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')]
            for tag, yaw in TAGS.items():
                fn = os.path.join(RES, 'dwb_r4f', f'sim_{s}_{tag}.jsonl')
                if not os.path.exists(fn):
                    check(f'{s}/{tag}: salida existe', False, fn)
                    continue
                L = [json.loads(z) for z in open(fn)]
                m = L[0]['meta']
                dk = sorted(k for k in set(meta_b) | set(m) if meta_b.get(k) != m.get(k))
                check(f'{s}/{tag}: meta == R4-E 0.15 salvo goal_yaw', dk == ['goal_yaw'], dk)
                check(f'{s}/{tag}: meta goal_yaw == {yaw}', abs(m['goal_yaw'] - yaw) < 1e-12, m.get('goal_yaw'))
                check(f'{s}/{tag}: mismos 64 runs y orden', [x['run'] for x in L[1:]] == [r[1] for r in runs_ref], len(L) - 1)
                check(f'{s}/{tag}: mismos goals', all(abs(x['goal'][i] - float(r[5 + i])) < 1e-9
                                                      for x, r in zip(L[1:], runs_ref) for i in range(3)))
    rep['PASS'] = bool(ok)
    os.makedirs(os.path.join(RES, 'dwb_r4f'), exist_ok=True)
    json.dump(rep, open(os.path.join(RES, 'dwb_r4f', 'run_verification.json' if a.runs else 'config_verification.json'), 'w'),
              indent=1, default=str)
    print('TODO PASS' if ok else 'HAY FALLAS')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
