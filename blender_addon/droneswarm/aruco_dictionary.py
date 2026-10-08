"""ArUco 'ORIGINAL' dictionary (5x5, ids 0..1023), the one Clover's aruco_map uses by default.

Each of the five marker rows stores two id bits as a 5-bit word, most significant pair first.
Pure Python, no Blender dependency.
"""

MARKER_BITS = 5
MAX_ID = 1023

_WORDS = (
    (1, 0, 0, 0, 0),  # 00
    (1, 0, 1, 1, 1),  # 01
    (0, 1, 0, 0, 1),  # 10
    (0, 1, 1, 1, 0),  # 11
)


def marker_grid(marker_id):
    """Cells of marker `marker_id`, border included, as rows from top to bottom.

    The result is a (MARKER_BITS + 2) square of 0 (black) and 1 (white) values.
    """
    if not 0 <= marker_id <= MAX_ID:
        raise ValueError(f"marker id must be in 0..{MAX_ID}, got {marker_id}")
    size = MARKER_BITS + 2
    grid = [[0] * size]
    for row in range(MARKER_BITS):
        pair = (marker_id >> (2 * (MARKER_BITS - 1 - row))) & 3
        grid.append([0, *_WORDS[pair], 0])
    grid.append([0] * size)
    return grid
