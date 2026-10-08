import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir, "droneswarm"))

from aruco_dictionary import marker_grid, MAX_ID
from aruco_map import MapMarker, MapParseError, format_map, grid_map, parse_map


class MapTest(unittest.TestCase):
    def test_parse(self):
        markers = parse_map("# comment\n\n0 0.33 0 0 0 0 0 0\n7\t0.2 1.5 -2 0.1 1.57 0 0  # tail\n")
        self.assertEqual(markers[0], MapMarker(0, 0.33))
        self.assertEqual(markers[1], MapMarker(7, 0.2, 1.5, -2, 0.1, 1.57))

    def test_parse_errors(self):
        for bad in ("x 0.3", "1", "1 0.3 0 0 0 0 0 0 0", "-1 0.3", "1 0"):
            with self.assertRaises(MapParseError):
                parse_map(bad)

    def test_roundtrip(self):
        markers = grid_map(0.33, 3, 2, 1, 0.5, first_id=10)
        self.assertEqual(parse_map(format_map(markers)), markers)

    def test_grid_top_left_first(self):
        top_left = grid_map(0.1, 2, 2, 1, 1)
        self.assertEqual((top_left[0].id, top_left[0].x, top_left[0].y), (0, 0, 1))
        bottom_left = grid_map(0.1, 2, 2, 1, 1, bottom_left=True)
        self.assertEqual((bottom_left[0].id, bottom_left[0].x, bottom_left[0].y), (0, 0, 0))


class DictionaryTest(unittest.TestCase):
    def test_grid_shape(self):
        grid = marker_grid(5)
        self.assertEqual(len(grid), 7)
        self.assertTrue(all(len(row) == 7 for row in grid))
        self.assertEqual(sum(grid[0]) + sum(grid[6]) + sum(r[0] + r[6] for r in grid), 0)

    def test_id_range(self):
        with self.assertRaises(ValueError):
            marker_grid(MAX_ID + 1)

    def test_matches_opencv(self):
        try:
            import cv2
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        except (ImportError, AttributeError):
            self.skipTest("OpenCV with aruco is not installed")
        for marker_id in range(MAX_ID + 1):
            image = cv2.aruco.drawMarker(dictionary, marker_id, 7, borderBits=1)
            self.assertEqual([[int(v > 127) for v in row] for row in image], marker_grid(marker_id), marker_id)


if __name__ == "__main__":
    unittest.main()
