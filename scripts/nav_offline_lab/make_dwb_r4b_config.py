#!/usr/bin/python3
"""NAV-OFFLINE-R4-B — configs de lazo cerrado derivadas de dwb_offline_R4A2_{base,fp}.yaml (sin tocar producción).

Cambios SOLO de infraestructura del simulador (DWB/critics/límites idénticos a A2):
  map_server.frame_id: map_true            (mapa del local costmap = geometría verdadera; el plan usa 'map' ruidoso)
  local_costmap.update_frequency: 0.0      (sin loop propio; el simulador llama updateMap() síncrono en cada paso)
  local_costmap.publish_frequency: 0.0     (sin publicar el grid)
"""
import copy, hashlib, json, os
import yaml
LAB = os.path.expanduser('~/limo_sml_ws/config/offline_lab')
man = {}
for v in ('base', 'fp'):
    src = os.path.join(LAB, f'dwb_offline_R4A2_{v}.yaml')
    raw = open(src, 'rb').read()
    d = yaml.safe_load(raw)
    o = copy.deepcopy(d)
    o['map_server']['ros__parameters']['frame_id'] = 'map_true'
    lp = o['local_costmap']['local_costmap']['ros__parameters']
    lp['update_frequency'] = 0.0
    lp['publish_frequency'] = 0.0
    assert o['controller_server'] == d['controller_server']
    fn = f'dwb_offline_R4B_{v}.yaml'
    txt = (f'# NAV-OFFLINE-R4-B — GENERADA por scripts/nav_offline_lab/make_dwb_r4b_config.py desde {os.path.basename(src)} '
           f'sha256={hashlib.sha256(raw).hexdigest()}\n# controller_server idéntico; solo map frame_id=map_true y costmap sin loop propio. '
           'NO usar en el robot.\n') + yaml.safe_dump(o, sort_keys=False, default_flow_style=None)
    open(os.path.join(LAB, fn), 'w').write(txt)
    man[fn] = hashlib.sha256(txt.encode()).hexdigest()
json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4B_manifest.json'), 'w'), indent=1)
print(man)
