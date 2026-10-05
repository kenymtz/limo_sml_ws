#!/usr/bin/python3
"""NAV-OFFLINE-R4-G — verificador: entre la candidata R4-F (yaw 0.10) y cada variante cambia SOLO un parámetro del
progress checker (required_movement_radius en PR*, movement_time_allowance en PT*).

  1. diff de dicts aplanados vs R4-F 0.10 == [controller_server...progress_checker.<param>] (completa y simulador)
  2. el resto de la candidata R4-F intacto: yaw 0.10, checker xy 0.25, FollowPath xy 0.15, RotateToGoal + ObstacleFootprint
  3. controller_server(simulador) == controller_server(candidata completa); producción con el sha256 de R4-D
  4. --runs: meta de dwb_r4g/sim_<S>_<TAG>.jsonl == meta de R4-F 0.10 salvo progress_radius / progress_time (== valor),
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
TAGS = {'PR005': ('required_movement_radius', 0.05, 'progress_radius'), 'PR003': ('required_movement_radius', 0.03, 'progress_radius'),
        'PT35': ('movement_time_allowance', 35.0, 'progress_time'), 'PT45': ('movement_time_allowance', 45.0, 'progress_time')}


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
    ap.add_argument('--tags', default=','.join(TAGS))
    a = ap.parse_args()
    tags = [t for t in a.tags.split(',') if t]
    rep, ok = dict(checks=[]), True

    def check(name, cond, detail=None):
        nonlocal ok
        ok &= bool(cond)
        rep['checks'].append(dict(check=name, PASS=bool(cond), detail=detail))
        print(('PASS ' if cond else 'FAIL ') + name + (f'  {detail}' if detail is not None and not cond else ''))

    real_sha = hashlib.sha256(open(REAL, 'rb').read()).hexdigest()
    check('produccion navigation2_real.yaml sin cambios',
          real_sha == json.load(open(os.path.join(LAB, 'dwb_offline_R4D_manifest.json')))['real_sha256'], real_sha)
    base = {'full': yaml.safe_load(open(os.path.join(LAB, 'navigation2_R4F_010.yaml'))),
            'sim': yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4F_010_sim.yaml')))}
    for tag in tags:
        key, val, _ = TAGS[tag]
        docs = {}
        for kind, name in (('full', f'navigation2_R4G_{tag}.yaml'), ('sim', f'dwb_offline_R4G_{tag}_sim.yaml')):
            d = docs[kind] = yaml.safe_load(open(os.path.join(LAB, name)))
            f = flat(d)
            K = CS + 'progress_checker.' + key
            dd = diff(base[kind], d)
            check(f'{name}: diff vs R4-F 0.10 == [{K}]', dd == [K], dd)
            check(f'{name}: {key} == {val}', f[K] == val, f[K])
            check(f'{name}: candidata R4-F intacta (yaw 0.10, checker xy 0.25, FollowPath xy 0.15, RTG+ObstacleFootprint)',
                  f[CS + 'general_goal_checker.yaw_goal_tolerance'] == 0.10 and f[CS + 'general_goal_checker.xy_goal_tolerance'] == 0.25
                  and f[CS + 'FollowPath.xy_goal_tolerance'] == 0.15
                  and {'RotateToGoal', 'ObstacleFootprint'} <= set(f[CS + 'FollowPath.critics']))
        check(f'R4G_{tag}: controller_server(simulador) == controller_server(candidata completa)',
              docs['full']['controller_server'] == docs['sim']['controller_server'])
    if a.runs:
        for s in SCEN:
            with open(os.path.join(RES, 'dwb_r4f', f'sim_{s}_010.jsonl')) as f:
                meta_b = json.loads(f.readline())['meta']
            runs_ref = [ln.split()[:9] for ln in open(os.path.join(RES, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')]
            for tag in tags:
                key, val, mk = TAGS[tag]
                fn = os.path.join(RES, 'dwb_r4g', f'sim_{s}_{tag}.jsonl')
                if not os.path.exists(fn):
                    check(f'{s}/{tag}: salida existe', False, fn)
                    continue
                L = [json.loads(z) for z in open(fn)]
                m = L[0]['meta']
                dk = sorted(k for k in set(meta_b) | set(m) if meta_b.get(k) != m.get(k))
                check(f'{s}/{tag}: meta == R4-F 0.10 salvo {mk}', dk == [mk], dk)
                check(f'{s}/{tag}: meta {mk} == {val}', abs(m[mk] - val) < 1e-12, m.get(mk))
                check(f'{s}/{tag}: mismos 64 runs y orden', [x['run'] for x in L[1:]] == [r[1] for r in runs_ref], len(L) - 1)
                check(f'{s}/{tag}: mismos goals', all(abs(x['goal'][i] - float(r[5 + i])) < 1e-9
                                                      for x, r in zip(L[1:], runs_ref) for i in range(3)))
    rep['PASS'] = bool(ok)
    os.makedirs(os.path.join(RES, 'dwb_r4g'), exist_ok=True)
    json.dump(rep, open(os.path.join(RES, 'dwb_r4g', 'run_verification.json' if a.runs else 'config_verification.json'), 'w'),
              indent=1, default=str)
    print('TODO PASS' if ok else 'HAY FALLAS')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
