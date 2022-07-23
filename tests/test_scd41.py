"""
test_scd41.py
"""

from my_package.utilities import crc8

def test_crc8():
    """
    Test CRC-8 algorithm
    """
    buf: bytearray = bytearray(2)
    buf[0] = 0x66 # MSB
    buf[1] = 0x67 # LSB
    assert crc8(buf) == 0xA2
