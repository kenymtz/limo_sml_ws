#!/usr/bin/python3
"""NAV-OFFLINE-R4-E — variantes de FollowPath.xy_goal_tolerance (la que usa RotateToGoal). NO producción.

Desde las configs de R4-D (NO se modifican) genera, para TOL en {0.20, 0.15, 0.10}:
  config/offline_lab/navigation2_R4E_<TAG>.yaml       candidata completa = navigation2_R4D.yaml con FollowPath.xy_goal_tolerance=TOL
  config/offline_lab/dwb_offline_R4E_<TAG>_sim.yaml   simulador          = dwb_offline_R4D_fp_rtg.yaml con el mismo cambio
general_goal_checker (0.25 / 0.25) NO cambia. Verificación de diff: verify_r4e_configs.py.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC_FULL = os.path.join(LAB, 'navigation2_R4D.yaml')
SRC_SIM = os.path.join(LAB, 'dwb_offline_R4D_fp_rtg.yaml')
VARIANTS = {'020': 0.20, '015': 0.15, '010': 0.10}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    man = dict(src_full=SRC_FULL, src_full_sha256=sha(open(SRC_FULL, 'rb').read()),
               src_sim=SRC_SIM, src_sim_sha256=sha(open(SRC_SIM, 'rb').read()), variants={})
    full0 = yaml.safe_load(open(SRC_FULL))
    sim0 = yaml.safe_load(open(SRC_SIM))
    for d in (full0, sim0):
        fp = d['controller_server']['ros__parameters']['FollowPath']
        assert fp['xy_goal_tolerance'] == 0.25 and 'RotateToGoal' in fp['critics'], fp
    for tag, tol in VARIANTS.items():
        out = {}
        for kind, src, d0, name in (('full', SRC_FULL, full0, f'navigation2_R4E_{tag}.yaml'),
                                    ('sim', SRC_SIM, sim0, f'dwb_offline_R4E_{tag}_sim.yaml')):
            d = copy.deepcopy(d0)
            d['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'] = tol
            hdr = (f'# NAV-OFFLINE-R4-E — GENERADA por scripts/nav_offline_lab/make_dwb_r4e_config.py. NO ES PRODUCCIÓN.\n'
                   f'# = {os.path.basename(src)} (sha256 {man["src_" + kind + "_sha256"][:16]}…) con ÚNICO cambio\n'
                   f'#   controller_server.FollowPath.xy_goal_tolerance: 0.25 -> {tol} (general_goal_checker sin cambios).\n')
            txt = hdr + yaml.safe_dump(d, sort_keys=False, default_flow_style=None)
            open(os.path.join(LAB, name), 'w').write(txt)
            out[kind] = dict(file=name, sha256=sha(txt.encode()))
        man['variants'][tag] = dict(xy_goal_tolerance=tol, **out)
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4E_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
