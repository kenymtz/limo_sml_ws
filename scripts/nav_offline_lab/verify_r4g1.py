#!/usr/bin/python3
"""NAV-OFFLINE-R4-G1 — verificaciones automáticas de la baseline replan_1hz.

  static   : (1) cada config *_replan = su config fija + SOLO GridBased.* y global_costmap.* (nada existente cambia);
             global_costmap == R3 salvo update/publish_frequency; F010 == dwb_offline_R4G0_replan_sim.yaml (validada en G0)
             (2) candidata F010 = R4-F: ObstacleFootprint+RotateToGoal, FollowPath xy 0.15, checker 0.25/0.10, progress 0.10/25
             (3) run_r4g1.sh usa los MISMOS parámetros de escenario que run_r4f_all.sh (solo agrega plan_mode=replan_1hz)
             (4) fuente del simulador contiene la restauración del global costmap (fix clearRobotCell) y el binario es más
                 nuevo que la fuente; (5) producción con el sha256 de R4-D
  selftest : sobre dwb_r4g1/selftest/sim_selftest.jsonl (S1, T005 T039 T041 T043 T045 en ese orden):
             (a) plan inicial NavFn == paths_R3.json (≤ 1e-5 m) en los 5 — con N1 recorrido antes por T039 (bug clearRobotCell)
             (b) cada corrida independiente de las anteriores: log de control idéntico al de la corrida R4-G0 v3 del mismo caso
                 (allí la precedían otros casos) -> costmap limpio por corrida
             (c) setPlan a 1 Hz; 0 fallos de NavFn; (d) auditoría del grafo PASS
  runs     : sobre dwb_r4g1/sim_<S>_<TAG>.jsonl: meta == meta R4-F/fuente + {plan_mode, replan_period} únicamente; mismos 64 runs,
             orden y goals; plan inicial == R3 en todas las corridas S1 (sin ruido); setPlan cada 1 s; auditorías PASS
"""
import glob
import hashlib
import json
import os
import re
import sys

import numpy as np
import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
RES = os.path.join(WS, 'results/navigation_offline_lab')
OUT = os.path.join(RES, 'dwb_r4g1')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')
SRC = {'B_A': 'dwb_offline_R4B_base.yaml', 'B_B': 'dwb_offline_R4B_fp.yaml', 'D': 'dwb_offline_R4D_fp_rtg.yaml',
       'E015': 'dwb_offline_R4E_015_sim.yaml', 'F010': 'dwb_offline_R4F_010_sim.yaml'}
FIXED_OUT = {'B_A': ('dwb_r4b', 'A'), 'B_B': ('dwb_r4b', 'B'), 'D': ('dwb_r4d', 'R'), 'E015': ('dwb_r4e', '015'), 'F010': ('dwb_r4f', '010')}
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
CS = 'controller_server.ros__parameters.'
WALL = {'wall_s', 'dwb_ms_mean', 'dwb_ms_max', 'updatemap_ms_mean', 'updatemap_ms_max'}


def flat(d, p=''):
    out = {}
    for k, v in d.items():
        kk = f'{p}.{k}' if p else str(k)
        if isinstance(v, dict):
            out.update(flat(v, kk))
        else:
            out[kk] = v
    return out


class Rep:
    def __init__(self):
        self.checks, self.ok = [], True

    def info(self, name, detail):
        self.checks.append(dict(check=name, PASS=None, INFO=detail))
        print('INFO ' + name + f'  {detail}')

    def __call__(self, name, cond, detail=None):
        self.ok &= bool(cond)
        self.checks.append(dict(check=name, PASS=bool(cond), detail=detail))
        print(('PASS ' if cond else 'FAIL ') + name + (f'  {detail}' if detail is not None and not cond else ''))


def sc_dict(fn):
    return dict(re.findall(r"\[(S\d_[a-z0-9_]+)\]='([^']*)'", open(fn).read()))


def static(check):
    check('produccion navigation2_real.yaml sin cambios',
          hashlib.sha256(open(REAL, 'rb').read()).hexdigest() ==
          json.load(open(os.path.join(LAB, 'dwb_offline_R4D_manifest.json')))['real_sha256'])
    r3 = yaml.safe_load(open(os.path.join(LAB, 'planner_offline_R3.yaml')))
    gc_r3 = flat(r3['global_costmap'], 'global_costmap')
    gb = flat(r3['planner_server']['ros__parameters']['GridBased'], CS + 'GridBased')
    for tag, src in SRC.items():
        a = flat(yaml.safe_load(open(os.path.join(LAB, src))))
        b = flat(yaml.safe_load(open(os.path.join(LAB, f'dwb_offline_R4G1_{tag}_replan_sim.yaml'))))
        dk = sorted(k for k in set(a) | set(b) if a.get(k, '<absent>') != b.get(k, '<absent>'))
        check(f'{tag}: solo se agregan GridBased.* / global_costmap.* a {src}',
              all(k.startswith(CS + 'GridBased.') or k.startswith('global_costmap.') for k in dk) and all(k not in a for k in dk), dk[:5])
        gdiff = sorted(k for k in set(gc_r3) | {k for k in b if k.startswith('global_costmap.')} if gc_r3.get(k) != b.get(k))
        check(f'{tag}: global_costmap == R3 salvo update/publish_frequency',
              gdiff == ['global_costmap.global_costmap.ros__parameters.publish_frequency',
                        'global_costmap.global_costmap.ros__parameters.update_frequency'], gdiff)
        check(f'{tag}: GridBased == R3', all(b.get(k) == v for k, v in gb.items()))
    f = yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4G1_F010_replan_sim.yaml')))
    g0 = yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4G0_replan_sim.yaml')))
    check('F010 replan == dwb_offline_R4G0_replan_sim.yaml (validada en R4-G0)', f == g0)
    c = flat(f)
    check('candidata R4-F: critics con ObstacleFootprint y RotateToGoal, sin BaseObstacle',
          {'ObstacleFootprint', 'RotateToGoal'} <= set(c[CS + 'FollowPath.critics']) and 'BaseObstacle' not in c[CS + 'FollowPath.critics'])
    check('candidata R4-F: FollowPath.xy_goal_tolerance 0.15', c[CS + 'FollowPath.xy_goal_tolerance'] == 0.15)
    check('candidata R4-F: checker xy 0.25 / yaw 0.10', c[CS + 'general_goal_checker.xy_goal_tolerance'] == 0.25
          and c[CS + 'general_goal_checker.yaw_goal_tolerance'] == 0.10)
    check('candidata R4-F: progress_checker 0.10 m / 25.0 s', c[CS + 'progress_checker.required_movement_radius'] == 0.1
          and c[CS + 'progress_checker.movement_time_allowance'] == 25.0)
    check('candidata R4-F: footprint B y padding 0.02', c['local_costmap.local_costmap.ros__parameters.footprint']
          == '[[-0.17, -0.16], [-0.17, 0.16], [0.22, 0.16], [0.22, -0.16]]' and c['local_costmap.local_costmap.ros__parameters.footprint_padding'] == 0.02)
    a, b = sc_dict(os.path.join(WS, 'scripts/nav_offline_lab/run_r4f_all.sh')), sc_dict(os.path.join(WS, 'scripts/nav_offline_lab/run_r4g1.sh'))
    check('escenarios de run_r4g1.sh == run_r4f_all.sh (6)', a == b and len(a) == 6, (a, b))
    srcf = os.path.join(WS, 'src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp')
    binf = os.path.join(WS, 'install/limo_nav_offline_lab/lib/limo_nav_offline_lab/dwb_closed_loop_sim')
    s = open(srcf).read()
    check('fuente con restauración del global costmap antes de cada plan (fix clearRobotCell)',
          'gpristine.assign' in s and 'std::copy(gpristine.begin(), gpristine.end(), cm);' in s
          and s.index('std::copy(gpristine.begin()') < s.index('np = navfn->createPlan(st, gl);'))
    check('binario más nuevo que la fuente', os.path.getmtime(binf) >= os.path.getmtime(srcf))
    return dict(binary_sha256=hashlib.sha256(open(binf, 'rb').read()).hexdigest(),
                source_sha256=hashlib.sha256(s.encode()).hexdigest())


def norm(x):
    y = {k: v for k, v in x.items() if k not in WALL and k not in ('setplan', 'n_setplan', 'n_plan_fail', 'plan0', 'g_restored_cells_cum', 'plan_mode')}
    y['ctrl'] = [r[:12] for r in x['ctrl']]
    return y


def selftest(check):
    paths = {r['test_id']: np.array(r['path'])[:, :2] for r in json.load(open(os.path.join(RES, 'planner_runs/paths_R3.json')))['results']}
    L = [json.loads(z) for z in open(os.path.join(OUT, 'selftest/sim_selftest.jsonl'))]
    runs = {x['run']: x for x in L[1:]}
    check('autotest: 5 corridas', sorted(runs) == ['T005', 'T039', 'T041', 'T043', 'T045'], sorted(runs))
    g0 = {x['run']: x for x in (json.loads(z) for z in list(open(os.path.join(RES, 'dwb_r4g0/sim_S1_nominal_replan_1hz.jsonl')))[1:])}
    cum = []
    for k, x in runs.items():
        p0 = np.array(x['plan0'])
        ref = paths[k]
        check(f'autotest {k}: plan inicial NavFn == paths_R3 (≤1e-5 m)', p0.shape == ref.shape and np.abs(p0 - ref).max() <= 1e-5,
              (p0.shape, ref.shape))
        check(f'autotest {k}: idéntico a R4-G0 v3 (otro orden de corridas previas) -> independencia/costmap limpio',
              norm(x) == norm(g0[k]))
        ts = [e[0] for e in x['setplan']]
        check(f'autotest {k}: setPlan cada 1.0 s y 0 fallos NavFn',
              x['n_plan_fail'] == 0 and np.allclose(np.diff(ts[1:]), 1.0, atol=1e-6))
        cum.append(x['g_restored_cells_cum'])
    check('autotest: celdas restauradas crecen (residuos de clearRobotCell detectados y eliminados)', cum == sorted(cum) and cum[-1] > 0, cum)
    for a in glob.glob(os.path.join(OUT, 'selftest/audit_*.json')):
        check(f'autotest: auditoría {os.path.basename(a)}', json.load(open(a))['PASS'])


def runs_check(check, tags):
    paths = {r['test_id']: np.array(r['path'])[:, :2] for r in json.load(open(os.path.join(RES, 'planner_runs/paths_R3.json')))['results']}
    for tag in tags:
        sub, suf = FIXED_OUT[tag]
        for s in SCEN:
            fn = os.path.join(OUT, f'sim_{s}_{tag}.jsonl')
            if not os.path.exists(fn):
                check(f'{tag}/{s}: salida existe', False, fn)
                continue
            L = [json.loads(z) for z in open(fn)]
            mb = json.loads(open(os.path.join(RES, sub, f'sim_{s}_{suf}.jsonl')).readline())['meta']
            m = L[0]['meta']
            dk = sorted(k for k in set(mb) | set(m) if mb.get(k) != m.get(k))
            check(f'{tag}/{s}: meta == fixed_plan salvo plan_mode/replan_period', dk == ['plan_mode', 'replan_period']
                  and m['plan_mode'] == 'replan_1hz' and m['replan_period'] == 1, dk)
            ref = [ln.split()[:9] for ln in open(os.path.join(RES, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')]
            X = L[1:]
            check(f'{tag}/{s}: 64 runs, orden y goals', [x['run'] for x in X] == [r[1] for r in ref]
                  and all(abs(x['goal'][i] - float(r[5 + i])) < 1e-9 for x, r in zip(X, ref) for i in range(3)))
            check(f'{tag}/{s}: setPlan (intentos de replan) cada 1.0 s', all(
                  np.allclose(np.diff([e[0] for e in x['setplan']][1:]), 1.0, atol=1e-6) for x in X))
            nf = {x['run']: x['n_plan_fail'] for x in X if x['n_plan_fail']}
            # INFO (no invalida la corrida): NavFn sin plan es un evento real del pipeline; en el BT real -> recovery
            check.info(f'{tag}/{s}: fallos de NavFn (INFO, ver informe)', nf)
            if s == 'S1_nominal':
                # tolerancia 1e-4 m: el JSON del simulador tiene 6 cifras significativas y paths_R3 es float32
                # (v1 de este check usaba 1e-5 y marcó T048/T050/T052/T054 con desvío 1.6e-5 m = misma geometría)
                bad = [x['run'] for x in X if not (np.array(x['plan0']).shape == paths[x['run']].shape
                                                    and np.abs(np.array(x['plan0']) - paths[x['run']]).max() <= 1e-4)]
                check(f'{tag}/{s}: plan inicial == paths_R3 en las 64 (sin ruido, ≤1e-4 m, mismo n de poses)', not bad, bad)
            a = os.path.join(OUT, f'audit_{s}_{tag}.json')
            check(f'{tag}/{s}: auditoría del grafo PASS (sin Twist/acciones)', os.path.exists(a) and json.load(open(a))['PASS'])


def main():
    mode = sys.argv[1]
    check = Rep()
    extra = {}
    if mode == 'static':
        extra = static(check)
    elif mode == 'selftest':
        selftest(check)
    elif mode == 'runs':
        runs_check(check, sys.argv[2].split(',') if len(sys.argv) > 2 else ['F010'])
    os.makedirs(OUT, exist_ok=True)
    json.dump(dict(mode=mode, checks=check.checks, PASS=check.ok, **extra),
              open(os.path.join(OUT, f'verify_{mode}.json'), 'w'), indent=1, default=str)
    print('TODO PASS' if check.ok else 'HAY FALLAS')
    sys.exit(0 if check.ok else 1)


if __name__ == '__main__':
    main()
