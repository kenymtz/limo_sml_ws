#!/usr/bin/python3
"""NAV-OFFLINE-R4-F — variantes de general_goal_checker.yaw_goal_tolerance. NO producción.

Desde las configs candidatas de R4-E 0.15 (NO se modifican) genera, para YAW en {0.20, 0.15, 0.10}:
  config/offline_lab/navigation2_R4F_<TAG>.yaml       = navigation2_R4E_015.yaml con general_goal_checker.yaw_goal_tolerance=YAW
  config/offline_lab/dwb_offline_R4F_<TAG>_sim.yaml   = dwb_offline_R4E_015_sim.yaml con el mismo cambio
general_goal_checker.xy_goal_tolerance (0.25) y FollowPath.xy_goal_tolerance (0.15) NO cambian. Verificación: verify_r4f_configs.py.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC_FULL = os.path.join(LAB, 'navigation2_R4E_015.yaml')
SRC_SIM = os.path.join(LAB, 'dwb_offline_R4E_015_sim.yaml')
VARIANTS = {'020': 0.20, '015': 0.15, '010': 0.10}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    man = dict(src_full=SRC_FULL, src_full_sha256=sha(open(SRC_FULL, 'rb').read()),
               src_sim=SRC_SIM, src_sim_sha256=sha(open(SRC_SIM, 'rb').read()), variants={})
    full0 = yaml.safe_load(open(SRC_FULL))
    sim0 = yaml.safe_load(open(SRC_SIM))
    for d in (full0, sim0):
        cs = d['controller_server']['ros__parameters']
        assert cs['general_goal_checker']['yaw_goal_tolerance'] == 0.25, cs['general_goal_checker']
        assert cs['general_goal_checker']['xy_goal_tolerance'] == 0.25 and cs['FollowPath']['xy_goal_tolerance'] == 0.15
    for tag, yaw in VARIANTS.items():
        out = {}
        for kind, src, d0, name in (('full', SRC_FULL, full0, f'navigation2_R4F_{tag}.yaml'),
                                    ('sim', SRC_SIM, sim0, f'dwb_offline_R4F_{tag}_sim.yaml')):
            d = copy.deepcopy(d0)
            d['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'] = yaw
            hdr = (f'# NAV-OFFLINE-R4-F — GENERADA por scripts/nav_offline_lab/make_dwb_r4f_config.py. NO ES PRODUCCIÓN.\n'
                   f'# = {os.path.basename(src)} (sha256 {man["src_" + kind + "_sha256"][:16]}…) con ÚNICO cambio\n'
                   f'#   controller_server.general_goal_checker.yaw_goal_tolerance: 0.25 -> {yaw}\n'
                   f'#   (general_goal_checker.xy_goal_tolerance 0.25 y FollowPath.xy_goal_tolerance 0.15 sin cambios).\n')
            txt = hdr + yaml.safe_dump(d, sort_keys=False, default_flow_style=None)
            open(os.path.join(LAB, name), 'w').write(txt)
            out[kind] = dict(file=name, sha256=sha(txt.encode()))
        man['variants'][tag] = dict(yaw_goal_tolerance=yaw, **out)
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4F_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
