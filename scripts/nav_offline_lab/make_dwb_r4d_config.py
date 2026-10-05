#!/usr/bin/python3
"""NAV-OFFLINE-R4-D — configs experimentales con RotateToGoal (NO producción).

  config/offline_lab/navigation2_R4D.yaml
      copia COMPLETA de config/navigation/navigation2_real.yaml con, en controller_server.FollowPath:
        - BaseObstacle -> ObstacleFootprint (footprint B, R4-A2; mismo scale 0.1)
        - RotateToGoal agregado a critics, en la posición de la línea comentada original
          (sus parámetros scale 32 / slowing_factor 5 / lookahead_time -1 ya existían en la config real)
      Nada más cambia (verificado por diff de dicts).
  config/offline_lab/dwb_offline_R4D_fp_rtg.yaml
      config del simulador = dwb_offline_R4B_fp.yaml + RotateToGoal. Assert: su controller_server es IDÉNTICO al de
      navigation2_R4D.yaml.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')
SIMB = os.path.join(LAB, 'dwb_offline_R4B_fp.yaml')


def flat(d, p=''):
    out = {}
    for k, v in d.items():
        kk = f'{p}.{k}' if p else k
        if isinstance(v, dict):
            out.update(flat(v, kk))
        else:
            out[kk] = v
    return out


def main():
    raw = open(REAL, 'rb').read()
    real = yaml.safe_load(raw)
    r4d = copy.deepcopy(real)
    fp = r4d['controller_server']['ros__parameters']['FollowPath']
    assert fp['critics'] == ['Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'], fp['critics']
    fp['critics'] = ['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']
    fp['ObstacleFootprint.scale'] = fp.pop('BaseObstacle.scale')
    assert all(k in fp for k in ('RotateToGoal.scale', 'RotateToGoal.slowing_factor', 'RotateToGoal.lookahead_time'))
    diff = sorted(k for k in set(flat(real)) | set(flat(r4d)) if flat(real).get(k) != flat(r4d).get(k))
    assert diff == ['controller_server.ros__parameters.FollowPath.BaseObstacle.scale',
                    'controller_server.ros__parameters.FollowPath.ObstacleFootprint.scale',
                    'controller_server.ros__parameters.FollowPath.critics'], diff
    hdr = ('# NAV-OFFLINE-R4-D — CONFIG EXPERIMENTAL (GENERADA por scripts/nav_offline_lab/make_dwb_r4d_config.py). NO ES PRODUCCIÓN.\n'
           f'# Fuente: config/navigation/navigation2_real.yaml sha256={hashlib.sha256(raw).hexdigest()} (NO modificado).\n'
           '# Cambios (únicos, verificados): FollowPath.critics += RotateToGoal (posición de la línea comentada original);\n'
           '#   BaseObstacle -> ObstacleFootprint (R4-A2), BaseObstacle.scale -> ObstacleFootprint.scale (0.1).\n'
           '# Los comentarios de la config original no se conservan (yaml.safe_dump).\n')
    txt = hdr + yaml.safe_dump(r4d, sort_keys=False, default_flow_style=None)
    open(os.path.join(LAB, 'navigation2_R4D.yaml'), 'w').write(txt)

    simb_raw = open(SIMB, 'rb').read()
    sim = yaml.safe_load(simb_raw)
    sfp = sim['controller_server']['ros__parameters']['FollowPath']
    sfp['critics'] = ['RotateToGoal'] + sfp['critics']
    assert sim['controller_server'] == r4d['controller_server'], 'controller_server del simulador != navigation2_R4D'
    stxt = ('# NAV-OFFLINE-R4-D — simulador: dwb_offline_R4B_fp.yaml + RotateToGoal (GENERADA). controller_server idéntico a '
            'navigation2_R4D.yaml. NO usar en el robot.\n') + yaml.safe_dump(sim, sort_keys=False, default_flow_style=None)
    open(os.path.join(LAB, 'dwb_offline_R4D_fp_rtg.yaml'), 'w').write(stxt)
    man = dict(real_sha256=hashlib.sha256(raw).hexdigest(), diff_vs_real=diff,
               navigation2_R4D_sha256=hashlib.sha256(txt.encode()).hexdigest(),
               sim_source=SIMB, sim_source_sha256=hashlib.sha256(simb_raw).hexdigest(),
               sim_sha256=hashlib.sha256(stxt.encode()).hexdigest(),
               critics=r4d['controller_server']['ros__parameters']['FollowPath']['critics'])
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4D_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
