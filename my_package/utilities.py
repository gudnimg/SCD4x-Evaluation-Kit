"""
Various helper functions for SCD41 package
"""

def crc8(data: bytearray) -> int:
    """
    CRC-8 algorithm
    """
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for bit in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = crc << 1
        crc &= 0xFF
    return crc
