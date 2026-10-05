"""Evaluación geométrica de footprints (offline). Obstáculo = OCUPADO ∪ UNKNOWN.

Grid fino: el mapa se sobre-muestrea x3 (0.03 -> 0.01 m) para reducir la cuantización
(el corredor N2 deja márgenes de milímetros). Error de discretización ~ ±0.5 cm.
"""
import os
import sys

import numpy as np
import yaml
from scipy import ndimage as ndi
from scipy.signal import fftconvolve

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_ros_map import read_image, classify_levels  # noqa

FOOTPRINTS = {
    'A': {'poly': np.array([[-0.16, -0.10], [-0.16, 0.10], [0.20, 0.10], [0.20, -0.10]]), 'padding': 0.03},
    'B': {'poly': np.array([[-0.17, -0.16], [-0.17, 0.16], [0.22, 0.16], [0.22, -0.16]]), 'padding': 0.02},
}
UP = 3


def pad_poly(poly, pad):
    """Igual que nav2_costmap_2d::padFootprint: suma pad con el signo de cada coordenada."""
    return poly + np.sign(poly) * pad


class GridMap:
    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path))
        img_path = meta['image'] if os.path.isabs(meta['image']) else \
            os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
        arr, _ = read_image(img_path)
        lv = np.unique(arr)
        cls = classify_levels(lv, meta.get('mode', 'trinary'), bool(meta.get('negate', 0)),
                              meta['occupied_thresh'], meta['free_thresh'])
        lut = np.zeros(256, np.int8)
        for px, c in cls.items():
            lut[px] = {'FREE': 0, 'OCCUPIED': 100}.get(c, -1)
        self.cls = np.flipud(lut[arr])                  # fila 0 = y mínimo
        self.res = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], float)
        self.h, self.w = self.cls.shape
        self.obst = self.cls != 0                       # ocupado o desconocido
        self.fres = self.res / UP
        self.obst_f = np.kron(self.obst, np.ones((UP, UP), bool))
        # distancia (m) desde el centro de cada celda fina al borde del obstáculo más cercano
        self.edt_f = ndi.distance_transform_edt(~self.obst_f) * self.fres - self.fres / 2
        self.edt_f[self.obst_f] = -self.fres / 2
        self.edt_c = ndi.distance_transform_edt(~self.obst) * self.res   # grid nativo (centro-centro)

    # ----- coordenadas -----
    def to_fine(self, xy):
        ij = np.floor((np.asarray(xy) - self.origin) / self.fres).astype(int)
        return ij[..., 1], ij[..., 0]                   # fila, columna

    def clearance_at(self, xy):
        r, c = self.to_fine(xy)
        ok = (r >= 0) & (r < self.edt_f.shape[0]) & (c >= 0) & (c < self.edt_f.shape[1])
        out = np.full(r.shape, -1.0)
        out[ok] = self.edt_f[r[ok], c[ok]]
        return out


def body_samples(poly, step=0.01):
    """Puntos (cuerpo) que cubren el rectángulo, borde incluido."""
    x0, y0 = poly.min(0); x1, y1 = poly.max(0)
    xs = np.r_[np.arange(x0, x1, step), x1]; ys = np.r_[np.arange(y0, y1, step), y1]
    X, Y = np.meshgrid(xs, ys)
    return np.c_[X.ravel(), Y.ravel()]


def footprint_clearance(gm, poses, poly):
    """poses Nx3 (x, y, yaw) -> clearance mínima (m) del polígono a obstáculo por pose (<=0 colisión)."""
    pts = body_samples(poly)
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    wx = poses[:, None, 0] + c[:, None] * pts[None, :, 0] - s[:, None] * pts[None, :, 1]
    wy = poses[:, None, 1] + s[:, None] * pts[None, :, 0] + c[:, None] * pts[None, :, 1]
    cl = gm.clearance_at(np.stack([wx, wy], -1))
    return cl.min(1)


def raster_kernel(poly, yaw, fres):
    """Máscara binaria del polígono rotado, centrada en el origen del robot."""
    c, s = np.cos(yaw), np.sin(yaw)
    P = poly @ np.array([[c, s], [-s, c]])
    R = np.abs(P).max() + fres
    n = int(np.ceil(R / fres))
    g = (np.arange(-n, n + 1)) * fres
    X, Y = np.meshgrid(g, g)
    inside = np.ones(X.shape, bool)
    for i in range(len(P)):                             # polígono convexo (orden horario/antihorario)
        a, b = P[i], P[(i + 1) % len(P)]
        cross = (b[0] - a[0]) * (Y - a[1]) - (b[1] - a[1]) * (X - a[0])
        inside &= cross <= 1e-9 if _is_cw(P) else cross >= -1e-9
    return inside


def _is_cw(P):
    return np.sum((P[1:, 0] - P[:-1, 0]) * (P[1:, 1] + P[:-1, 1])) + \
        (P[0, 0] - P[-1, 0]) * (P[0, 1] + P[-1, 1]) > 0


def cspace_free(gm, poly, yaws):
    """Devuelve array (n_yaw, H_f, W_f) bool: True = el centro puede estar ahí con ese yaw sin colisión."""
    out = np.empty((len(yaws),) + gm.obst_f.shape, bool)
    ob = gm.obst_f.astype(np.float32)
    for k, yaw in enumerate(yaws):
        K = raster_kernel(poly, yaw, gm.fres).astype(np.float32)
        # C-obstáculo: centro q colisiona si existe obstáculo en q + R(yaw)·p  ->  correlación con K
        hit = fftconvolve(ob, K[::-1, ::-1], mode='same') > 0.5
        out[k] = ~hit
    return out
