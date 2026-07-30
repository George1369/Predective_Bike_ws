import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bike_msgs'))

from radar_node.radar_driver_node import parse_distance_sample, parse_range_state


def test_parse_distance_sample():
    sample = "distance:1.42"
    distance = parse_distance_sample(sample)
    assert distance == pytest.approx(1.42, abs=1e-6)


def test_parse_distance_sample_invalid():
    distance = parse_distance_sample("not-a-distance")
    assert distance is None


def test_parse_range_state():
    sample = "range:distance:1.42;presence:1;motion:0;signal_strength:0.87;confidence:0.91"
    state = parse_range_state(sample)

    assert state is not None
    distance, presence, motion, signal_strength, confidence = state
    assert distance == pytest.approx(1.42, abs=1e-6)
    assert presence is True
    assert motion is False
    assert signal_strength == pytest.approx(0.87, abs=1e-6)
    assert confidence == pytest.approx(0.91, abs=1e-6)
