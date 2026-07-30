import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from display_node.gui_display_node import build_osm_tile_url
from display_node.screen_display_node import build_map_overlay


def test_build_map_overlay():
    overlay = build_map_overlay(48.1173, 11.5167, 5.5)
    assert 'MAP' in overlay
    assert '48.1173' in overlay
    assert '11.5167' in overlay
    assert '5.50 m/s' in overlay


def test_build_map_overlay_marks_hazards_and_route():
    overlay = build_map_overlay(48.1173, 11.5167, 5.5, obstacle=True)
    assert 'SIMULATED MAP' in overlay
    assert 'Route' in overlay
    assert 'Hazard' in overlay


def test_build_osm_tile_url_uses_coordinates_and_zoom():
    url = build_osm_tile_url(48.1173, 11.5167, zoom=15)
    assert 'tile.openstreetmap.org/15/' in url
    assert url.endswith('.png')
