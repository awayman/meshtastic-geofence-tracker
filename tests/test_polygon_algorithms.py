#!/usr/bin/env python3
"""
test_polygon_algorithms.py
Unit tests for the point-in-polygon algorithm embedded in meshtastic-geofence.lua.

Tests are implemented in Python using the same ray-casting logic so they can
run in CI without Meshtastic hardware.  The Python implementation mirrors the
Lua source exactly so any logic change must be applied to both.

Run:
    python3 tests/test_polygon_algorithms.py
    # or
    pytest tests/test_polygon_algorithms.py -v
"""

import re
import unittest


# ---------------------------------------------------------------------------
# Python mirror of the Lua point_in_polygon function
# ---------------------------------------------------------------------------

def point_in_polygon(px: float, py: float, polygon: list) -> bool:
    """
    Ray-casting point-in-polygon test.

    Args:
        px: latitude of the query point
        py: longitude of the query point
        polygon: list of [lat, lon] vertex pairs (at least 3)

    Returns:
        True if (px, py) is strictly inside the polygon.
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1  # previous vertex index (0-based, wraps)

    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        crosses = ((yi > py) != (yj > py)) and \
                  (px < (xj - xi) * (py - yi) / (yj - yi) + xi)
        if crosses:
            inside = not inside

        j = i

    return inside


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestPointInPolygon(unittest.TestCase):

    # ------------------------------------------------------------------
    # Rectangle: corners at (0,0), (0,2), (2,2), (2,0)   (lat, lon)
    # ------------------------------------------------------------------

    RECT = [
        [0.0, 0.0],
        [0.0, 2.0],
        [2.0, 2.0],
        [2.0, 0.0],
    ]

    def test_rect_centre_inside(self):
        self.assertTrue(point_in_polygon(1.0, 1.0, self.RECT))

    def test_rect_off_centre_inside(self):
        self.assertTrue(point_in_polygon(0.5, 0.5, self.RECT))

    def test_rect_near_edge_inside(self):
        self.assertTrue(point_in_polygon(1.99, 1.99, self.RECT))

    def test_rect_outside_right(self):
        self.assertFalse(point_in_polygon(1.0, 3.0, self.RECT))

    def test_rect_outside_left(self):
        self.assertFalse(point_in_polygon(1.0, -1.0, self.RECT))

    def test_rect_outside_above(self):
        self.assertFalse(point_in_polygon(3.0, 1.0, self.RECT))

    def test_rect_outside_below(self):
        self.assertFalse(point_in_polygon(-1.0, 1.0, self.RECT))

    def test_rect_far_outside(self):
        self.assertFalse(point_in_polygon(100.0, 100.0, self.RECT))

    # ------------------------------------------------------------------
    # Degenerate polygons
    # ------------------------------------------------------------------

    def test_empty_polygon(self):
        self.assertFalse(point_in_polygon(1.0, 1.0, []))

    def test_single_vertex(self):
        self.assertFalse(point_in_polygon(1.0, 1.0, [[1.0, 1.0]]))

    def test_two_vertices(self):
        self.assertFalse(point_in_polygon(1.0, 1.0, [[0.0, 0.0], [2.0, 2.0]]))

    # ------------------------------------------------------------------
    # Triangle
    # ------------------------------------------------------------------

    # Triangle with vertices (0,0), (4,0), (2,4)
    TRIANGLE = [
        [0.0, 0.0],
        [4.0, 0.0],
        [2.0, 4.0],
    ]

    def test_triangle_centroid_inside(self):
        # Centroid approximately (2, 4/3)
        self.assertTrue(point_in_polygon(2.0, 1.33, self.TRIANGLE))

    def test_triangle_outside_left(self):
        self.assertFalse(point_in_polygon(0.0, 2.0, self.TRIANGLE))

    def test_triangle_outside_right(self):
        self.assertFalse(point_in_polygon(4.0, 2.0, self.TRIANGLE))

    def test_triangle_above_apex(self):
        self.assertFalse(point_in_polygon(2.0, 5.0, self.TRIANGLE))

    # ------------------------------------------------------------------
    # Real-world GPS coordinates (home yard rectangle)
    # Polygon: SW=(40.7128,-74.0060), NW=(40.7135,-74.0060),
    #          NE=(40.7135,-74.0055), SE=(40.7128,-74.0055)
    # ------------------------------------------------------------------

    HOME_YARD = [
        [40.7128, -74.0060],
        [40.7135, -74.0060],
        [40.7135, -74.0055],
        [40.7128, -74.0055],
    ]

    def test_gps_centre_inside(self):
        self.assertTrue(point_in_polygon(40.71315, -74.00575, self.HOME_YARD))

    def test_gps_corner_proximity_inside(self):
        # Just inside the SW corner
        self.assertTrue(point_in_polygon(40.71285, -74.00595, self.HOME_YARD))

    def test_gps_outside_west(self):
        self.assertFalse(point_in_polygon(40.71315, -74.0065, self.HOME_YARD))

    def test_gps_outside_east(self):
        self.assertFalse(point_in_polygon(40.71315, -74.0050, self.HOME_YARD))

    def test_gps_outside_north(self):
        self.assertFalse(point_in_polygon(40.7140, -74.00575, self.HOME_YARD))

    def test_gps_outside_south(self):
        self.assertFalse(point_in_polygon(40.7120, -74.00575, self.HOME_YARD))

    # ------------------------------------------------------------------
    # Irregular convex polygon (hexagon-like)
    # ------------------------------------------------------------------

    HEXAGON = [
        [2.0, 0.0],
        [4.0, 1.0],
        [4.0, 3.0],
        [2.0, 4.0],
        [0.0, 3.0],
        [0.0, 1.0],
    ]

    def test_hexagon_centre_inside(self):
        self.assertTrue(point_in_polygon(2.0, 2.0, self.HEXAGON))

    def test_hexagon_outside_corner(self):
        self.assertFalse(point_in_polygon(5.0, 5.0, self.HEXAGON))

    def test_hexagon_near_edge_outside(self):
        self.assertFalse(point_in_polygon(0.0, 0.0, self.HEXAGON))

    # ------------------------------------------------------------------
    # Concave (L-shaped) polygon
    # ------------------------------------------------------------------

    L_SHAPE = [
        [0.0, 0.0],
        [0.0, 4.0],
        [2.0, 4.0],
        [2.0, 2.0],
        [4.0, 2.0],
        [4.0, 0.0],
    ]

    def test_l_shape_bottom_left_inside(self):
        self.assertTrue(point_in_polygon(1.0, 1.0, self.L_SHAPE))

    def test_l_shape_top_left_inside(self):
        self.assertTrue(point_in_polygon(1.0, 3.0, self.L_SHAPE))

    def test_l_shape_bottom_right_inside(self):
        self.assertTrue(point_in_polygon(3.0, 1.0, self.L_SHAPE))

    def test_l_shape_notch_outside(self):
        # The notch area (top-right of the L)
        self.assertFalse(point_in_polygon(3.0, 3.0, self.L_SHAPE))


# ---------------------------------------------------------------------------
# JSON field extraction tests (mirrors Lua json_* helpers)
# ---------------------------------------------------------------------------

def lua_json_str(text, key):
    m = re.search(r'"' + re.escape(key) + r'"\s*:\s*"([^"]*)"', text)
    return m.group(1) if m else None


def lua_json_num(text, key):
    m = re.search(r'"' + re.escape(key) + r'"\s*:\s*(-?\d+\.?\d*)', text)
    return float(m.group(1)) if m else None


def lua_json_bool(text, key):
    m = re.search(r'"' + re.escape(key) + r'"\s*:\s*(true|false)', text)
    return m.group(1) == "true" if m else False


def lua_json_polygon(text):
    m = re.search(r'"polygon"\s*:\s*(\[\[.*?\]\])', text, re.DOTALL)
    if not m:
        return None
    arr_str = m.group(1)
    verts = re.findall(r'\[\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\]', arr_str)
    polygon = [[float(lat), float(lon)] for lat, lon in verts]
    return polygon if len(polygon) >= 3 else None


SAMPLE_CONFIG = (
    '{"type":"geofence_config","tracker_id":"1234","person_name":"Alice",'
    '"geofence_name":"home_yard",'
    '"polygon":[[40.7128,-74.0060],[40.7135,-74.0060],[40.7135,-74.0055],[40.7128,-74.0055]],'
    '"relay_gpio":10,"enable_beeper":true,"version":1}'
)


class TestJsonParsing(unittest.TestCase):

    def test_parse_type(self):
        self.assertEqual(lua_json_str(SAMPLE_CONFIG, "type"), "geofence_config")

    def test_parse_tracker_id(self):
        self.assertEqual(lua_json_str(SAMPLE_CONFIG, "tracker_id"), "1234")

    def test_parse_person_name(self):
        self.assertEqual(lua_json_str(SAMPLE_CONFIG, "person_name"), "Alice")

    def test_parse_geofence_name(self):
        self.assertEqual(lua_json_str(SAMPLE_CONFIG, "geofence_name"), "home_yard")

    def test_parse_relay_gpio(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "relay_gpio"), 10.0)

    def test_parse_enable_beeper_true(self):
        self.assertTrue(lua_json_bool(SAMPLE_CONFIG, "enable_beeper"))

    def test_parse_enable_beeper_false(self):
        msg = SAMPLE_CONFIG.replace('"enable_beeper":true', '"enable_beeper":false')
        self.assertFalse(lua_json_bool(msg, "enable_beeper"))

    def test_parse_version(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "version"), 1.0)

    def test_parse_polygon_vertex_count(self):
        poly = lua_json_polygon(SAMPLE_CONFIG)
        self.assertIsNotNone(poly)
        self.assertEqual(len(poly), 4)

    def test_parse_polygon_first_vertex(self):
        poly = lua_json_polygon(SAMPLE_CONFIG)
        self.assertAlmostEqual(poly[0][0], 40.7128)
        self.assertAlmostEqual(poly[0][1], -74.0060)

    def test_parse_polygon_last_vertex(self):
        poly = lua_json_polygon(SAMPLE_CONFIG)
        self.assertAlmostEqual(poly[3][0], 40.7128)
        self.assertAlmostEqual(poly[3][1], -74.0055)

    def test_polygon_too_few_vertices(self):
        bad_msg = '{"type":"geofence_config","polygon":[[1.0,2.0],[3.0,4.0]]}'
        self.assertIsNone(lua_json_polygon(bad_msg))

    def test_missing_polygon_key(self):
        bad_msg = '{"type":"geofence_config","relay_gpio":10}'
        self.assertIsNone(lua_json_polygon(bad_msg))


if __name__ == "__main__":
    unittest.main(verbosity=2)
