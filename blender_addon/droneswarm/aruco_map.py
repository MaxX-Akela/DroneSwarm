"""Clover ArUco map files: `id size x y z z_angle y_angle x_angle` per line, angles in radians.

Pure Python, no Blender dependency.
"""
from dataclasses import dataclass


class MapParseError(ValueError):
    pass


@dataclass
class MapMarker:
    id: int
    size: float
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rot_z: float = 0.0
    rot_y: float = 0.0
    rot_x: float = 0.0


def parse_map(text):
    """Parse the contents of a map file. Blank lines and '#' comments are skipped."""
    markers = []
    for number, line in enumerate(text.splitlines(), start=1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if not 2 <= len(fields) <= 8:
            raise MapParseError(f"line {number}: expected 'id size [x y z z_angle y_angle x_angle]', got {len(fields)} fields")
        try:
            marker_id = int(fields[0])
            values = [float(v) for v in fields[1:]]
        except ValueError as e:
            raise MapParseError(f"line {number}: {e}") from None
        if marker_id < 0:
            raise MapParseError(f"line {number}: negative marker id")
        if values[0] <= 0:
            raise MapParseError(f"line {number}: marker size must be positive")
        markers.append(MapMarker(marker_id, *values))
    return markers


def _num(value):
    return f"{value:.6f}".rstrip("0").rstrip(".") or "0"


def format_map(markers):
    lines = ["# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x"]
    for m in markers:
        fields = (m.size, m.x, m.y, m.z, m.rot_z, m.rot_y, m.rot_x)
        lines.append("\t".join([str(m.id), *(_num(v) for v in fields)]))
    return "\n".join(lines) + "\n"


def grid_map(size, count_x, count_y, dist_x, dist_y, first_id=0, bottom_left=False, z=0.0):
    """Markers on a regular grid; ids grow along x, row by row.

    The first marker is the top-left one unless `bottom_left` is set. Marker centers start at (0, 0).
    """
    if count_x < 1 or count_y < 1:
        raise ValueError("marker counts must be positive")
    markers = []
    for row in range(count_y):
        y_index = row if bottom_left else count_y - 1 - row
        for col in range(count_x):
            markers.append(MapMarker(first_id + row * count_x + col, size,
                                     col * dist_x, y_index * dist_y, z))
    return markers
