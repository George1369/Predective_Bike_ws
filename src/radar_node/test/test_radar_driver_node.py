import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bike_msgs'))

from radar_node.radar_driver_node import (
    DISTANCE_CALIBRATION_NEEDED,
    DISTANCE_COUNT_MASK,
    DISTANCE_ERROR,
    signed_u32,
)


def test_signed_u32_handles_positive_and_negative_strengths():
    assert signed_u32(1250) == 1250
    assert signed_u32(0xFFFFF830) == -2000


def test_distance_result_flags_match_the_a121_register_layout():
    result = 3 | DISTANCE_CALIBRATION_NEEDED
    assert result & DISTANCE_COUNT_MASK == 3
    assert result & DISTANCE_CALIBRATION_NEEDED
    assert not result & DISTANCE_ERROR
