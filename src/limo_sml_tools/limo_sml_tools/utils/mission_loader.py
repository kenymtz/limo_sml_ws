import math
from pathlib import Path

import yaml


class MissionConfigError(Exception):
    """Error en la configuración de una misión SML."""


def _require(mapping, key, context):
    if key not in mapping:
        raise MissionConfigError(
            f"Falta '{key}' en {context}"
        )
    return mapping[key]


def load_mission(path):
    path = Path(path).expanduser()

    if not path.is_file():
        raise MissionConfigError(
            f"No existe el archivo de misión: {path}"
        )

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise MissionConfigError(
            "El YAML de misión debe contener un diccionario principal."
        )

    arena = _require(data, "arena", "la misión")

    map_config = _require(data, "map", "la misión")
    start_config = _require(data, "start", "la misión")
    stations_config = _require(data, "stations", "la misión")

    if not isinstance(map_config, dict):
        raise MissionConfigError("'map' debe ser un diccionario.")

    if not isinstance(start_config, dict):
        raise MissionConfigError("'start' debe ser un diccionario.")

    if not isinstance(stations_config, dict):
        raise MissionConfigError("'stations' debe ser un diccionario.")

    map_name = _require(map_config, "name", "map")
    map_frame = _require(map_config, "frame", "map")

    start = {
        "x": float(_require(start_config, "x", "start")),
        "y": float(_require(start_config, "y", "start")),
        "yaw_deg": float(
            _require(start_config, "yaw_deg", "start")
        ),
    }

    start["yaw"] = math.radians(start["yaw_deg"])

    stations = {}

    for name, station in stations_config.items():

        if not isinstance(station, dict):
            raise MissionConfigError(
                f"La estación '{name}' debe ser un diccionario."
            )

        yaw_deg = float(
            _require(station, "yaw_deg", f"stations.{name}")
        )

        stations[name] = {
            "x": float(
                _require(station, "x", f"stations.{name}")
            ),
            "y": float(
                _require(station, "y", f"stations.{name}")
            ),
            "yaw_deg": yaw_deg,
            "yaw": math.radians(yaw_deg),
        }

    return {
        "arena": str(arena),
        "map": {
            "name": str(map_name),
            "frame": str(map_frame),
        },
        "start": start,
        "stations": stations,
    }
