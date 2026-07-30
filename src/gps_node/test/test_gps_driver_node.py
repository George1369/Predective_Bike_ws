import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gps_node.gps_driver_node import parse_nmea_sentence


def test_parse_gprmc_sentence():
    line = "$GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"

    lat, lon, alt, speed_mps, has_fix = parse_nmea_sentence(line)

    assert has_fix is True
    assert lat == pytest.approx(48.1173, abs=1e-4)
    assert lon == pytest.approx(11.5166667, abs=1e-4)
    assert speed_mps == pytest.approx(11.52, abs=1e-2)
    assert alt == pytest.approx(0.0, abs=1e-9)


def test_parse_gga_sentence():
    line = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"

    lat, lon, alt, speed_mps, has_fix = parse_nmea_sentence(line)

    assert has_fix is True
    assert lat == pytest.approx(48.1173, abs=1e-4)
    assert lon == pytest.approx(11.5166667, abs=1e-4)
    assert alt == pytest.approx(545.4, abs=1e-9)
    assert speed_mps == pytest.approx(0.0, abs=1e-9)
