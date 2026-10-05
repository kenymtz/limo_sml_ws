#!/usr/bin/python3
"""FASE 4 — mapa SINTÉTICO de desarrollo (NO representa el entorno físico).

Geometría (m, frame map). Todo alineado a celdas de 0.03 m (coordenadas = k*0.03):
  - interior libre x,y ∈ [0, 9.99] (333 celdas = 9.99 m), paredes de 0.09 m por fuera
  - UNKNOWN fuera del perímetro (margen 0.72 m)
  - divisor vertical en x ∈ [4.95, 5.04] con 3 pasos (corredores de 1.50 m de largo, x ∈ [4.26, 5.76]):
      WIDE : ancho 1.20 m, centro y≈2.01
      N1   : ancho 0.45 m, centro y≈5.01
      N2   : ancho 0.33 m, centro y≈8.01
  - obstáculos: caja O1, caja O2 (rectangular), columna O3 (círculo), pared en L O4
Salidas en results/navigation_offline_lab/synthetic_arena/:
  synthetic_arena.pgm / .yaml / .png, synthetic_arena_geometry.json
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

OUT = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab/synthetic_arena')
RES = 0.03
MARGIN_C = 24                 # 0.72 m de UNKNOWN exterior (+ pared)
WALL_C = 3                    # 0.09 m
INT_C = 333                   # 9.99 m interior
N = MARGIN_C * 2 + WALL_C * 2 + INT_C   # 387 celdas
ORIGIN = (-(MARGIN_C + WALL_C) * RES, -(MARGIN_C + WALL_C) * RES)  # (-0.81, -0.81)

FREE, OCC, UNK = 254, 0, 205


def c(v):
    """metros (frame map) -> índice de celda (esquina inferior)."""
    return int(round((v - ORIGIN[0]) / RES))


grid = np.full((N, N), UNK, np.uint8)          # fila 0 = y mínimo (se voltea al escribir)


def rect(x0, x1, y0, y1, val):
    grid[c(y0):c(y1), c(x0):c(x1)] = val


# interior + paredes
rect(-0.09, 9.99 + 0.09, -0.09, 9.99 + 0.09, OCC)
rect(0.0, 9.99, 0.0, 9.99, FREE)

geom = {'resolution': RES, 'origin': [round(ORIGIN[0], 4), round(ORIGIN[1], 4), 0],
        'size_px': [N, N], 'interior': [0.0, 9.99, 0.0, 9.99], 'wall_thickness': 0.09,
        'corridors': {}, 'obstacles': {},
        'disclaimer': 'MAPA SINTETICO DE DESARROLLO. No representa el entorno fisico.'}

# divisor con 3 pasos
DX0, DX1 = 4.95, 5.04
CX0, CX1 = 4.26, 5.76          # largo del corredor 1.50 m
passages = {'WIDE': (1.41, 2.61), 'N1': (4.785, 5.235), 'N2': (7.845, 8.175)}
# ajustar a celdas exactas
passages = {k: (round(round(a / RES) * RES, 4), round(round(a / RES) * RES + round((b - a) / RES) * RES, 4))
            for k, (a, b) in passages.items()}
rect(DX0, DX1, 0.0, 9.99, OCC)
for name, (y0, y1) in passages.items():
    rect(DX0, DX1, y0, y1, FREE)                       # abrir el paso
    rect(CX0, CX1, y0 - 0.09, y0, OCC)                 # pared lateral inferior
    rect(CX0, CX1, y1, y1 + 0.09, OCC)                 # pared lateral superior
    geom['corridors'][name] = {'x': [CX0, CX1], 'y': [y0, y1], 'width_m': round(y1 - y0, 4),
                               'length_m': round(CX1 - CX0, 4), 'axis': 'x'}

# obstáculos
rect(1.50, 2.52, 6.51, 7.50, OCC); geom['obstacles']['O1_box'] = {'x': [1.50, 2.52], 'y': [6.51, 7.50]}
rect(7.20, 7.80, 3.00, 4.80, OCC); geom['obstacles']['O2_box'] = {'x': [7.20, 7.80], 'y': [3.00, 4.80]}
# columna circular r=0.30 en (2.49, 3.99)
yy, xx = np.mgrid[0:N, 0:N]
cxw, cyw = ORIGIN[0] + (xx + 0.5) * RES, ORIGIN[1] + (yy + 0.5) * RES
grid[(cxw - 2.49) ** 2 + (cyw - 3.99) ** 2 <= 0.30 ** 2] = OCC
geom['obstacles']['O3_column'] = {'center': [2.49, 3.99], 'radius': 0.30}
# pared en L
rect(6.99, 8.49, 6.51, 6.60, OCC); rect(8.40, 8.49, 6.51, 8.49, OCC)
geom['obstacles']['O4_L_wall'] = {'segments': [[6.99, 8.49, 6.51, 6.60], [8.40, 8.49, 6.51, 8.49]]}
geom['corners'] = [[0.0, 0.0], [9.99, 0.0], [9.99, 9.99], [0.0, 9.99]]

os.makedirs(OUT, exist_ok=True)
img = np.flipud(grid)                               # fila 0 de la imagen = y máximo
with open(os.path.join(OUT, 'synthetic_arena.pgm'), 'wb') as f:
    f.write(f'P5\n{N} {N}\n255\n'.encode())
    f.write(img.tobytes())
with open(os.path.join(OUT, 'synthetic_arena.yaml'), 'w') as f:
    f.write('# MAPA SINTETICO DE DESARROLLO - NO representa el entorno fisico (NAV-OFFLINE-R2)\n'
            'image: synthetic_arena.pgm\nmode: trinary\n'
            f'resolution: {RES}\norigin: [{ORIGIN[0]:.2f}, {ORIGIN[1]:.2f}, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.10\n')
json.dump(geom, open(os.path.join(OUT, 'synthetic_arena_geometry.json'), 'w'), indent=1)

ext = [ORIGIN[0], ORIGIN[0] + N * RES, ORIGIN[1], ORIGIN[1] + N * RES]
shade = np.where(img == FREE, 1.0, np.where(img == OCC, 0.0, 0.7))
fig, ax = plt.subplots(figsize=(9, 9))
ax.imshow(shade, cmap='gray', vmin=0, vmax=1, extent=ext, origin='upper')
for name, cd in geom['corridors'].items():
    ax.annotate(f'{name} {cd["width_m"]:.2f} m', (cd['x'][1] + 0.1, np.mean(cd['y'])), color='tab:blue', fontsize=10,
                va='center')
for name, o in geom['obstacles'].items():
    p = o.get('center') or [np.mean(o['x']), np.mean(o['y'])] if 'x' in o else o.get('center', [7.7, 7.5])
    ax.annotate(name, p, color='tab:red', fontsize=9, ha='center')
ax.set_title('synthetic_arena — MAPA SINTÉTICO (no representa el entorno físico)\n'
             'blanco libre · negro ocupado · gris UNKNOWN · res 0.03 m')
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(alpha=0.25)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'synthetic_arena.png'), dpi=110)
print(json.dumps({k: geom[k] for k in ('origin', 'size_px', 'corridors')}, indent=1))
