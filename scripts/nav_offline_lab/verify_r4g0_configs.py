#!/usr/bin/python3
"""NAV-OFFLINE-R4-G0 — verificador de la config de replanning.

  1. dwb_offline_R4G0_replan_sim.yaml vs dwb_offline_R4F_010_sim.yaml: las únicas claves nuevas/distintas son
     controller_server...GridBased.* y global_costmap.* (todo FollowPath, checkers, progress checker y local_costmap idénticos)
  2. global_costmap == planner_offline_R3.yaml global_costmap salvo update_frequency / publish_frequency (0, arnés)
  3. GridBased == planner_server.GridBased de R3
  4. producción con el sha256 de R4-D
"""
import hashlib
import json
import os
import sys

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
RES = os.path.join(WS, 'results/navigation_offline_lab')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')


def flat(d, p=''):
    out = {}
    for k, v in d.items():
        kk = f'{p}.{k}' if p else str(k)
        if isinstance(v, dict):
            out.update(flat(v, kk))
        else:
            out[kk] = v
    return out


def main():
    rep, ok = dict(checks=[]), True

    def check(name, cond, detail=None):
        nonlocal ok
        ok &= bool(cond)
        rep['checks'].append(dict(check=name, PASS=bool(cond), detail=detail))
        print(('PASS ' if cond else 'FAIL ') + name + (f'  {detail}' if detail is not None and not cond else ''))

    check('produccion navigation2_real.yaml sin cambios',
          hashlib.sha256(open(REAL, 'rb').read()).hexdigest() ==
          json.load(open(os.path.join(LAB, 'dwb_offline_R4D_manifest.json')))['real_sha256'])
    a = flat(yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4F_010_sim.yaml'))))
    b = flat(yaml.safe_load(open(os.path.join(LAB, 'dwb_offline_R4G0_replan_sim.yaml'))))
    r3 = yaml.safe_load(open(os.path.join(LAB, 'planner_offline_R3.yaml')))
    dk = sorted(k for k in set(a) | set(b) if a.get(k, '<absent>') != b.get(k, '<absent>'))
    other = [k for k in dk if not (k.startswith('controller_server.ros__parameters.GridBased.') or k.startswith('global_costmap.'))]
    check('solo se agregan GridBased.* y global_costmap.* (resto idéntico a R4-F 0.10)', not other, other)
    check('ninguna clave existente de R4-F cambia', all(k not in a for k in dk), [k for k in dk if k in a])
    gc_r3 = flat(r3['global_costmap'], 'global_costmap')
    gdiff = sorted(k for k in set(gc_r3) | {k for k in b if k.startswith('global_costmap.')} if gc_r3.get(k) != b.get(k))
    check('global_costmap == R3 salvo update/publish_frequency',
          gdiff == ['global_costmap.global_costmap.ros__parameters.publish_frequency',
                    'global_costmap.global_costmap.ros__parameters.update_frequency'], gdiff)
    gb = flat(r3['planner_server']['ros__parameters']['GridBased'], 'controller_server.ros__parameters.GridBased')
    check('GridBased == planner_server.GridBased de R3', all(b.get(k) == v for k, v in gb.items())
          and len([k for k in b if k.startswith('controller_server.ros__parameters.GridBased.')]) == len(gb))
    rep['PASS'] = bool(ok)
    os.makedirs(os.path.join(RES, 'dwb_r4g0'), exist_ok=True)
    json.dump(rep, open(os.path.join(RES, 'dwb_r4g0', 'config_verification.json'), 'w'), indent=1, default=str)
    print('TODO PASS' if ok else 'HAY FALLAS')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
