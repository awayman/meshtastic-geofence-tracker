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

import math
import re
import unittest


# ---------------------------------------------------------------------------
# Python mirror of the Lua point_in_polygon function
# ---------------------------------------------------------------------------

EPSILON = 1e-9
EARTH_RADIUS_M = 6371000  # mean Earth radius in metres


def point_on_segment(px: float, py: float, xi: float, yi: float, xj: float, yj: float) -> bool:
    cross = (px - xi) * (yj - yi) - (py - yi) * (xj - xi)
    if abs(cross) > EPSILON:
        return False

    dot = (px - xi) * (px - xj) + (py - yi) * (py - yj)
    return dot <= EPSILON


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

        if point_on_segment(px, py, xi, yi, xj, yj):
            return False

        crosses = ((yi > py) != (yj > py)) and \
                  (px < (xj - xi) * (py - yi) / (yj - yi) + xi)
        if crosses:
            inside = not inside

        j = i

    return inside


# ---------------------------------------------------------------------------
# Python mirror of the Lua distance-to-boundary functions
# ---------------------------------------------------------------------------

def point_to_segment_dist_m(px: float, py: float,
                             x1: float, y1: float,
                             x2: float, y2: float) -> float:
    """
    Shortest distance in metres from point (px, py) to line segment
    (x1, y1)→(x2, y2), where all coordinates are in degrees (lat, lon).

    Uses a flat-Earth (equirectangular) projection accurate to ±5 m for
    distances under ~1 km.
    """
    mid_lat = (x1 + x2) / 2.0
    cos_lat = math.cos(math.radians(mid_lat))
    m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * cos_lat

    ax = (x2 - x1) * m_per_deg_lat
    ay = (y2 - y1) * m_per_deg_lon
    bx = (px - x1) * m_per_deg_lat
    by = (py - y1) * m_per_deg_lon

    seg_len_sq = ax * ax + ay * ay

    if seg_len_sq < EPSILON:
        return math.sqrt(bx * bx + by * by)

    t = (bx * ax + by * ay) / seg_len_sq
    t = max(0.0, min(1.0, t))
    dx = bx - t * ax
    dy = by - t * ay
    return math.sqrt(dx * dx + dy * dy)


def dist_to_boundary_m(px: float, py: float, polygon: list):
    """
    Signed distance in metres from (px, py) to the nearest polygon edge.

    Returns:
        positive float  – inside geofence (distance to closest edge)
        negative float  – outside geofence (negated distance to closest edge)
        None            – polygon has fewer than 3 vertices
    """
    n = len(polygon)
    if n < 3:
        return None

    min_dist = math.inf
    j = n - 1
    for i in range(n):
        d = point_to_segment_dist_m(
            px, py,
            polygon[i][0], polygon[i][1],
            polygon[j][0], polygon[j][1],
        )
        if d < min_dist:
            min_dist = d
        j = i

    inside = point_in_polygon(px, py, polygon)
    return min_dist if inside else -min_dist


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

    def test_rect_point_on_edge_outside(self):
        self.assertFalse(point_in_polygon(1.0, 0.0, self.RECT))

    def test_rect_point_on_vertex_outside(self):
        self.assertFalse(point_in_polygon(0.0, 0.0, self.RECT))

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
# Distance-to-boundary tests
# ---------------------------------------------------------------------------

class TestDistToBoundary(unittest.TestCase):

    # Square centred near NYC (~78m × ~62m in metres at this latitude)
    # SW=(40.7128,-74.0060) → NE=(40.7135,-74.0055)
    HOME_YARD = [
        [40.7128, -74.0060],
        [40.7135, -74.0060],
        [40.7135, -74.0055],
        [40.7128, -74.0055],
    ]

    def test_degenerate_returns_none(self):
        self.assertIsNone(dist_to_boundary_m(0.0, 0.0, []))
        self.assertIsNone(dist_to_boundary_m(0.0, 0.0, [[0.0, 0.0], [1.0, 1.0]]))

    def test_outside_returns_negative(self):
        # Clearly outside to the west
        d = dist_to_boundary_m(40.71315, -74.0070, self.HOME_YARD)
        self.assertIsNotNone(d)
        self.assertLess(d, 0.0)

    def test_inside_returns_positive(self):
        # Centre of the yard
        d = dist_to_boundary_m(40.71315, -74.00575, self.HOME_YARD)
        self.assertIsNotNone(d)
        self.assertGreater(d, 0.0)

    def test_inside_centre_reasonable_magnitude(self):
        # The yard is ~78m lat × ~62m lon; the inradius should be ~31m
        d = dist_to_boundary_m(40.71315, -74.00575, self.HOME_YARD)
        self.assertGreater(d, 5.0)
        self.assertLess(d, 50.0)

    def test_outside_magnitude_reasonable(self):
        # Point ~110 m south (≈0.001° lat) of the southern edge
        d = dist_to_boundary_m(40.7118, -74.00575, self.HOME_YARD)
        self.assertLess(d, 0.0)
        self.assertGreater(abs(d), 50.0)
        self.assertLess(abs(d), 200.0)

    def test_sign_matches_point_in_polygon(self):
        # Any inside point should give positive dist, outside → negative
        pts_in  = [(40.71315, -74.00575), (40.71290, -74.00580)]
        pts_out = [(40.7140,  -74.00575), (40.71315, -74.0070)]
        for lat, lon in pts_in:
            d = dist_to_boundary_m(lat, lon, self.HOME_YARD)
            self.assertGreater(d, 0.0,
                msg=f"Expected positive dist for inside point ({lat},{lon}), got {d}")
        for lat, lon in pts_out:
            d = dist_to_boundary_m(lat, lon, self.HOME_YARD)
            self.assertLess(d, 0.0,
                msg=f"Expected negative dist for outside point ({lat},{lon}), got {d}")

    # ------------------------------------------------------------------
    # point_to_segment_dist_m unit tests
    # ------------------------------------------------------------------

    def test_perpendicular_to_segment(self):
        # Point directly above the midpoint of a horizontal 1-degree segment
        # at the equator: (0,0)→(0,1), query (0.001, 0.5)
        # Distance ≈ 0.001 * m_per_deg_lat ≈ 111.195 m
        d = point_to_segment_dist_m(0.001, 0.5, 0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(d, 0.001 * EARTH_RADIUS_M * math.pi / 180.0, delta=1.0)

    def test_beyond_endpoint_clamps(self):
        # Point beyond end of segment should clamp to endpoint distance
        d_end  = point_to_segment_dist_m(0.0, 2.0, 0.0, 0.0, 0.0, 1.0)
        d_far  = point_to_segment_dist_m(0.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        self.assertGreater(d_far, d_end)

    def test_zero_length_segment(self):
        # Zero-length segment: distance equals distance to the endpoint
        d = point_to_segment_dist_m(0.001, 0.001, 0.0, 0.0, 0.0, 0.0)
        # Should be finite positive numbers
        self.assertGreater(d, 0.0)
        self.assertLess(d, 1000.0)

    # ------------------------------------------------------------------
    # zone_from_dist logic (mirrors Lua zone_from_dist)
    # ------------------------------------------------------------------

    def zone_from_dist(self, dist_m,
                       approaching=50, near=20, critical=5):
        """Python mirror of Lua zone_from_dist."""
        if dist_m is None:
            return None
        if dist_m <= 0:
            return "OUTSIDE"
        if dist_m <= critical:
            return "CRITICAL"
        if dist_m <= near:
            return "NEAR"
        if dist_m <= approaching:
            return "APPROACHING"
        return "SAFE"

    def test_zone_safe(self):
        self.assertEqual(self.zone_from_dist(100.0), "SAFE")

    def test_zone_approaching(self):
        self.assertEqual(self.zone_from_dist(50.0), "APPROACHING")
        self.assertEqual(self.zone_from_dist(25.0), "APPROACHING")

    def test_zone_near(self):
        self.assertEqual(self.zone_from_dist(20.0), "NEAR")
        self.assertEqual(self.zone_from_dist(10.0), "NEAR")

    def test_zone_critical(self):
        self.assertEqual(self.zone_from_dist(5.0), "CRITICAL")
        self.assertEqual(self.zone_from_dist(0.1), "CRITICAL")

    def test_zone_outside(self):
        self.assertEqual(self.zone_from_dist(0.0), "OUTSIDE")
        self.assertEqual(self.zone_from_dist(-1.0), "OUTSIDE")
        self.assertEqual(self.zone_from_dist(-100.0), "OUTSIDE")

    def test_zone_none(self):
        self.assertIsNone(self.zone_from_dist(None))

    def test_zone_real_gps_inside(self):
        d = dist_to_boundary_m(40.71315, -74.00575, self.HOME_YARD)
        zone = self.zone_from_dist(d)
        self.assertIn(zone, ("SAFE", "APPROACHING", "NEAR", "CRITICAL"))

    def test_zone_real_gps_outside(self):
        d = dist_to_boundary_m(40.7140, -74.00575, self.HOME_YARD)
        self.assertEqual(self.zone_from_dist(d), "OUTSIDE")


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
    return (m.group(1) == "true") if m else None


def lua_json_polygon(text):
    m = re.search(r'"polygon"\s*:\s*', text)
    if not m or m.end() >= len(text) or text[m.end()] != "[":
        return None

    start = m.end()
    depth = 0
    end = None
    for idx in range(start, len(text)):
        ch = text[idx]
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
            if depth == 0:
                end = idx
                break

    if end is None:
        return None

    arr_str = text[start:end + 1]
    verts = re.findall(r'\[\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\]', arr_str)
    polygon = [[float(lat), float(lon)] for lat, lon in verts]
    return polygon if len(polygon) >= 3 else None


SAMPLE_CONFIG = (
    '{"type":"geofence_config","tracker_id":"1234","person_name":"Alice",'
    '"geofence_name":"home_yard",'
    '"polygon":[[40.7128,-74.0060],[40.7135,-74.0060],[40.7135,-74.0055],[40.7128,-74.0055]],'
    '"relay_gpio":10,"enable_beeper":true,"buzzer_gpio":0,'
    '"warn_dist_approaching":50,"warn_dist_near":20,"warn_dist_critical":5,"version":1}'
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

    def test_parse_enable_beeper_missing(self):
        msg = SAMPLE_CONFIG.replace('"enable_beeper":true,', "")
        self.assertIsNone(lua_json_bool(msg, "enable_beeper"))

    def test_parse_buzzer_gpio(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "buzzer_gpio"), 0.0)

    def test_parse_warn_dist_approaching(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "warn_dist_approaching"), 50.0)

    def test_parse_warn_dist_near(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "warn_dist_near"), 20.0)

    def test_parse_warn_dist_critical(self):
        self.assertEqual(lua_json_num(SAMPLE_CONFIG, "warn_dist_critical"), 5.0)

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

    def test_parse_polygon_with_whitespace(self):
        msg = (
            '{\n'
            '  "type": "geofence_config",\n'
            '  "polygon": [ [40.7128, -74.0060], [40.7135, -74.0060],\n'
            '               [40.7135, -74.0055], [40.7128, -74.0055] ]\n'
            '}'
        )
        poly = lua_json_polygon(msg)
        self.assertIsNotNone(poly)
        self.assertEqual(len(poly), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)


# ---------------------------------------------------------------------------
# Python mirror of the Lua point_in_polygon function
# ---------------------------------------------------------------------------

EPSILON = 1e-9


def point_on_segment(px: float, py: float, xi: float, yi: float, xj: float, yj: float) -> bool:
    cross = (px - xi) * (yj - yi) - (py - yi) * (xj - xi)
    if abs(cross) > EPSILON:
        return False

    dot = (px - xi) * (px - xj) + (py - yi) * (py - yj)
    return dot <= EPSILON


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

        if point_on_segment(px, py, xi, yi, xj, yj):
            return False

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

    def test_rect_point_on_edge_outside(self):
        self.assertFalse(point_in_polygon(1.0, 0.0, self.RECT))

    def test_rect_point_on_vertex_outside(self):
        self.assertFalse(point_in_polygon(0.0, 0.0, self.RECT))

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
    return (m.group(1) == "true") if m else None


def lua_json_polygon(text):
    m = re.search(r'"polygon"\s*:\s*', text)
    if not m or m.end() >= len(text) or text[m.end()] != "[":
        return None

    start = m.end()
    depth = 0
    end = None
    for idx in range(start, len(text)):
        ch = text[idx]
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
            if depth == 0:
                end = idx
                break

    if end is None:
        return None

    arr_str = text[start:end + 1]
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

    def test_parse_enable_beeper_missing(self):
        msg = SAMPLE_CONFIG.replace('"enable_beeper":true,', "")
        self.assertIsNone(lua_json_bool(msg, "enable_beeper"))

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

    def test_parse_polygon_with_whitespace(self):
        msg = (
            '{\n'
            '  "type": "geofence_config",\n'
            '  "polygon": [ [40.7128, -74.0060], [40.7135, -74.0060],\n'
            '               [40.7135, -74.0055], [40.7128, -74.0055] ]\n'
            '}'
        )
        poly = lua_json_polygon(msg)
        self.assertIsNotNone(poly)
        self.assertEqual(len(poly), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
