#!/usr/bin/env python3
"""
Analisis offline (solo lectura) de clearance para comparar dos footprints
candidatos del LIMO sobre un mapa de occupancy grid ya existente.

No publica en ROS, no mueve el robot, no modifica ningun archivo de
configuracion. Solo lee un .pgm/.yaml de mapa y calcula distancia a
obstaculos (distance transform) para clasificar zonas donde el
footprint "grande" (navigation2_real.yaml actual) requiere mas
espacio del que el footprint "medido" (base_motion_r1) necesitaria.

Uso:
    python3 footprint_clearance_analysis.py <ruta_mapa.pgm> <resolucion_m>
"""

import os
import sys
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

# Todas las salidas (imagenes, resultados) se escriben unicamente aqui.
# Nunca dentro de ~/limoatwork, sin importar de donde venga el mapa de entrada.
OUTPUT_DIR = os.path.expanduser("~/limo_sml_ws/results/footprint_analysis")

# Footprints candidatos (medios de ancho/largo, en metros, incluyendo
# footprint_padding aplicado como offset uniforme sobre cada extremo).
# Fuente: results/baseline/base_motion_r1/RESULTS.txt (medido)
#         config/navigation/navigation2_real.yaml (actual)
FOOTPRINT_MEDIDO = {
    "x_min": -0.16 - 0.03, "x_max": 0.20 + 0.03,
    "y_min": -0.10 - 0.03, "y_max": 0.10 + 0.03,
}
FOOTPRINT_ACTUAL = {
    "x_min": -0.17 - 0.02, "x_max": 0.22 + 0.02,
    "y_min": -0.16 - 0.02, "y_max": 0.16 + 0.02,
}


def half_width(fp):
    return max(abs(fp["y_min"]), abs(fp["y_max"]))


def half_length(fp):
    return max(abs(fp["x_min"]), abs(fp["x_max"]))


def bounding_radius(fp):
    corners = [
        (fp["x_min"], fp["y_min"]), (fp["x_min"], fp["y_max"]),
        (fp["x_max"], fp["y_min"]), (fp["x_max"], fp["y_max"]),
    ]
    return max((x ** 2 + y ** 2) ** 0.5 for x, y in corners)


def main():
    if len(sys.argv) != 3:
        print("uso: footprint_clearance_analysis.py <mapa.pgm> <resolucion_m>")
        sys.exit(1)

    map_path = sys.argv[1]
    resolution = float(sys.argv[2])

    img = np.array(Image.open(map_path))
    free = img >= 250          # convencion map_server: 254 = libre
    occupied = img <= 5        # 0 = ocupado
    unknown = ~free & ~occupied  # 205 = desconocido

    # Para un analisis conservador de seguridad, tratamos "desconocido"
    # como no transitable (igual que occupied) al calcular distancia.
    non_free = occupied | unknown
    dist_px = distance_transform_edt(~non_free)
    dist_m = dist_px * resolution

    rm = half_width(FOOTPRINT_MEDIDO)
    ra = half_width(FOOTPRINT_ACTUAL)
    rad_m = bounding_radius(FOOTPRINT_MEDIDO)
    rad_a = bounding_radius(FOOTPRINT_ACTUAL)

    pasa_medido = free & (dist_m >= rm)
    pasa_actual = free & (dist_m >= ra)
    solo_medido_pasa = pasa_medido & ~pasa_actual

    total_free_px = int(free.sum())
    n_pasa_medido = int(pasa_medido.sum())
    n_pasa_actual = int(pasa_actual.sum())
    n_solo_medido = int(solo_medido_pasa.sum())

    cell_area = resolution * resolution

    print("=== Analisis de clearance offline (footprint vs mapa) ===")
    print(f"Mapa: {map_path}")
    print(f"Tamano: {img.shape[1]}x{img.shape[0]} px, resolucion {resolution} m/px")
    print(f"Area total del mapa: {img.shape[0]*img.shape[1]*cell_area:.2f} m^2")
    print()
    print("Footprint MEDIDO (base_motion_r1, con padding 0.03):")
    print(f"  semi-ancho (y) = {rm:.3f} m -> ancho total = {2*rm:.3f} m")
    print(f"  radio circunscrito = {rad_m:.3f} m")
    print()
    print("Footprint ACTUAL (navigation2_real.yaml, con padding 0.02):")
    print(f"  semi-ancho (y) = {ra:.3f} m -> ancho total = {2*ra:.3f} m")
    print(f"  radio circunscrito = {rad_a:.3f} m")
    print()
    print(f"Celdas libres en el mapa: {total_free_px} ({total_free_px*cell_area:.2f} m^2)")
    print(f"Celdas transitables por footprint MEDIDO: {n_pasa_medido} "
          f"({100*n_pasa_medido/max(total_free_px,1):.1f}% del espacio libre)")
    print(f"Celdas transitables por footprint ACTUAL: {n_pasa_actual} "
          f"({100*n_pasa_actual/max(total_free_px,1):.1f}% del espacio libre)")
    print(f"Celdas transitables SOLO por el footprint MEDIDO "
          f"(bloqueadas para el ACTUAL): {n_solo_medido} "
          f"({100*n_solo_medido/max(total_free_px,1):.1f}% del espacio libre, "
          f"{n_solo_medido*cell_area:.3f} m^2)")
    print()
    print("NOTA METODOLOGICA:")
    print("- Se usa una aproximacion por semi-ancho isotropico (clearance")
    print("  circular), NO un chequeo SE(2) exacto de colision del")
    print("  poligono completo en cada orientacion. Es conservador para")
    print("  identificar CANDIDATOS a zonas problematicas, no un veredicto")
    print("  definitivo de colision.")
    print("- 'Desconocido' (205) se trata como no transitable (conservador).")

    # Guardar mapa de clasificacion como PNG para inspeccion visual offline.
    # Se escribe siempre en OUTPUT_DIR (nunca junto al mapa de entrada), para
    # garantizar que ninguna escritura caiga dentro de ~/limoatwork.
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    map_basename = os.path.splitext(os.path.basename(map_path))[0]
    out_path = os.path.join(OUTPUT_DIR, f"{map_basename}_footprint_comparacion.png")
    rgb = np.zeros((*img.shape, 3), dtype=np.uint8)
    rgb[occupied] = (0, 0, 0)
    rgb[unknown] = (128, 128, 128)
    rgb[pasa_actual] = (0, 200, 0)          # pasa con ambos footprints
    rgb[solo_medido_pasa] = (255, 165, 0)   # solo pasa con el medido (bloqueado por el actual)
    rgb[free & ~pasa_medido & ~pasa_actual] = (200, 0, 0)  # no pasa con ninguno
    Image.fromarray(rgb).save(out_path)
    print(f"\nImagen comparativa guardada (solo lectura de entrada, nuevo archivo): {out_path}")
    print("  verde = transitable con ambos footprints")
    print("  naranja = transitable SOLO con el footprint medido (bloqueado por el actual)")
    print("  rojo = no transitable con ninguno de los dos")
    print("  gris = desconocido, negro = ocupado")


if __name__ == "__main__":
    main()
