import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from display_node.screen_display_node import format_display_text


def test_format_display_text():
    data = {
        'gps_lat': 48.1173,
        'gps_lon': 11.5167,
        'gps_speed': 5.5,
        'radar_distance': 1.42,
        'radar_presence': True,
        'radar_motion': False,
        'radar_confidence': 0.91,
    }

    text = format_display_text(data)

    assert 'GPS' in text
    assert '48.1173' in text
    assert '11.5167' in text
    assert '1.42 m' in text
    assert 'PRESENT' in text
    assert '0.91' in text
