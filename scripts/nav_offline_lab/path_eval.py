"""Evaluación de rutas (módulo sin efectos al importar). Misma lógica que
evaluate_planner_benchmark.py (R2): ver ese docstring para el modelo de ejecución
SUPUESTO (giro en sitio en start/goal, rumbo tangente con ventana 0.10 m) y las clases
PASS / MARGINAL / COLLISION (MARGIN_LOC = 0.05 m, valor de trabajo NO medido).
"""
import numpy as np

from footprint_eval import FOOTPRINTS, footprint_clearance

MARGIN_LOC = 0.05
HEAD_WIN = 0.10


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot_seq(x, y, a0, a1, step=np.deg2rad(5)):
    d = wrap(a1 - a0)
    n = max(1, int(np.ceil(abs(d) / step)))
    return np.array([[x, y, a0 + d * k / n] for k in range(n + 1)])


def densify(P, step=0.01):
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        L = np.hypot(*(b - a))
        n = max(1, int(np.ceil(L / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.array(out)


def headings(P):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    h = np.zeros(len(P))
    for i in range(len(P)):
        j = np.searchsorted(d, d[i] + HEAD_WIN)
        if j >= len(P):
            j = len(P) - 1
        k = i
        if j == i:
            k = max(0, np.searchsorted(d, d[i] - HEAD_WIN) - 1); j = i
        v = P[j] - P[k]
        h[i] = np.arctan2(v[1], v[0]) if np.hypot(*v) > 1e-6 else (h[i - 1] if i else 0.0)
    return h


def passage_used(P, geo):
    sel = (P[:, 0] > 4.95) & (P[:, 0] < 5.04)
    used = set()
    for y in P[sel, 1]:
        for name, cd in geo['corridors'].items():
            if cd['y'][0] <= y <= cd['y'][1]:
                used.add(name)
    return '+'.join(sorted(used)) if used else 'none'


def classify(c, pad):
    return 'COLLISION' if c <= 0 else ('MARGINAL' if c < pad + MARGIN_LOC else 'PASS')


def evaluate_result(r, gm, geo, cfg):
    """r: registro de paths_*.json -> (fila dict, poses swept, P)."""
    s, g = np.array(r['start']), np.array(r['goal'])
    row = {'test_id': r['test_id'], 'category': r['category'], 'planner_config': cfg,
           'status': r['status'], 'path_found': r['status'] == 'SUCCEEDED' and len(r['path']) > 1,
           'planning_time_s': r['planning_time_s'], 'poses': len(r['path'])}
    if not row['path_found']:
        return row, None, None
    P = np.array(r['path'])[:, :2]
    L = float(np.sum(np.hypot(*np.diff(P, axis=0).T)))
    cc = gm.clearance_at(P)
    D = densify(P)
    H = headings(D)
    trans = np.c_[D, H]
    rs = rot_seq(D[0, 0], D[0, 1], s[2], H[0])
    rg = rot_seq(D[-1, 0], D[-1, 1], H[-1], g[2])
    row.update(path_length_m=round(L, 4), goal_err_m=round(float(np.hypot(*(P[-1] - g[:2]))), 4),
               minimum_clearance=round(float(cc.min()), 4), mean_clearance=round(float(cc.mean()), 4),
               p05_clearance=round(float(np.percentile(cc, 5)), 4), passage=passage_used(P, geo),
               cells_in_unknown=int((gm.cls[((P[:, 1] - gm.origin[1]) / gm.res).astype(int),
                                            ((P[:, 0] - gm.origin[0]) / gm.res).astype(int)] == -1).sum()))
    for F, spec in FOOTPRINTS.items():
        ct = footprint_clearance(gm, trans, spec['poly'])
        cs = footprint_clearance(gm, rs, spec['poly'])
        cgl = footprint_clearance(gm, rg, spec['poly'])
        m_all = float(min(ct.min(), cs.min(), cgl.min()))
        row[f'{F}_min_fp_clearance_m'] = round(m_all, 4)
        row[f'{F}_class'] = classify(m_all, spec['padding'])
    return row, np.r_[rs, trans, rg], P


def path_deviation(P1, P2):
    """Hausdorff simétrico (m) entre dos polilíneas (densificadas a 1 cm)."""
    from scipy.spatial import cKDTree
    a, b = densify(P1), densify(P2)
    return float(max(cKDTree(b).query(a)[0].max(), cKDTree(a).query(b)[0].max()))
