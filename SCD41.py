import time
from machine import SoftI2C

# I2C commands - Basic commands
START_PERIODIC_MEASUREMENT = 0x21B1
READ_MEASUREMENT = 0xEC05
STOP_PERIODIC_MEASUREMENT = 0x3F86

# I2C commands - On-chip output signal compensation
SET_TEMPERATURE_OFFSET = 0x241D
GET_TEMPERATURE_OFFSET = 0x2318
SET_SENSOR_ALTITUDE = 0x2427
GET_SENSOR_ALTITUDE = 0x2322
SET_AMBIENT_PRESSURE = 0xE000

# I2C commands - Field calibration
PERFORM_FORCED_RECALIBRATION = 0x362F
SET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2416
GET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2313

# I2C commands - Low power
START_LOW_POWER_PERIODIC_MEASUREMENT = 0x21AC
GET_DATA_READY_STATUS = 0xE4B8

# I2C commands - Advanced features
PERSIST_SETTINGS = 0x3615
GET_SERIAL_NUMBER = 0x3682
PERFORM_SELF_TEST = 0x3639
PERFORM_FACTORY_RESET = 0x3632
PERFORM_REINIT = 0x3646

# I2c commands - Low power single shot
MEASURE_SINGLE_SHOT = 0x219D
MEASURE_SINGLE_SHOT_RHT_ONLY = 0x2196
POWER_DOWN = 0x36E0
WAKE_UP = 0x36F6

isIdle: bool = True


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


def start_periodic_measurement(i2c: SoftI2C) -> None:
    """
    Start periodic measurement, signal update interval is 5 seconds.
    """
    global isIdle
    i2c.writeto_mem(0x62, START_PERIODIC_MEASUREMENT, b"", addrsize=16)
    isIdle = False


def read_measurement(i2c: SoftI2C) -> None:

    buf: bytearray = bytearray(9)

    start_periodic_measurement(i2c)

    while is_data_ready(i2c) == False:
        continue

    i2c.readfrom_mem_into(0x62, READ_MEASUREMENT, buf, addrsize=16)

    CO2_ppm = buf[0] << 8 | buf[1]
    if crc8(buf[0:2]) != buf[2]:
        print(
            "Received CRC %s did not match calculated CRC %s"
            % (hex(buf[2]), hex(crc8(buf[0:2])))
        )
    print("CO2: %sppm" % CO2_ppm)

    temperature = buf[3] << 8 | buf[4]
    temperature = -45 + (175 * temperature) / (2**16 - 1)
    if crc8(buf[3:5]) != buf[5]:
        print(
            "Received CRC %s did not match calculated CRC %s"
            % (hex(buf[5]), hex(crc8(buf[3:5])))
        )
    print("Temperature: %.2f°C" % temperature)

    relative_humidity = buf[6] << 8 | buf[7]
    relative_humidity = (100 * relative_humidity) / (2**16 - 1)
    if crc8(buf[6:8]) != buf[8]:
        print(
            "Received CRC %s did not match calculated CRC %s"
            % (hex(buf[8]), hex(crc8(buf[6:8])))
        )
    print("Humidity: %.2f%%" % relative_humidity)


def stop_periodic_measurement(i2c: SoftI2C) -> None:
    """
    Stop periodic measurement to change the sensor configuration or to save power.
    Note that the sensor will only respond to other commands after waiting 500 ms
    after issuing the stop_periodic_measurement command.
    """
    global isIdle
    i2c.writeto_mem(0x62, STOP_PERIODIC_MEASUREMENT, b"", addrsize=16)
    isIdle = True
    time.sleep(0.5)  # Sensor becomes responsive after 500ms


def is_data_ready(i2c: SoftI2C) -> bool:
    buf: bytearray = bytearray(3)
    i2c.readfrom_mem_into(0x62, GET_DATA_READY_STATUS, buf, addrsize=16)
    return ((buf[0] & 0x7) << 8) | buf[1] != 0


# TODO
# def set_temperature_offset(i2c: SoftI2C) -> None:

# TODO
# def get_temperature_offset(i2c: SoftI2C) -> None:


def set_sensor_altitude(i2c: SoftI2C, altitude: int) -> None:
    """
    Get the set sensor altitude. The altitude represenets meters above sea level.
    NOTE: Sensor must be in IDLE mode
    """
    if isIdle == False:
        print("# Error: Sensor is not IDLE")
        return

    buf: bytearray = bytearray(3)
    altitude_bytes = altitude.to_bytes(2, "big")
    buf[0] = altitude_bytes[0]  # MSB
    buf[1] = altitude_bytes[1]  # LSB
    buf[2] = crc8(buf[0:2])  # CRC
    i2c.writeto_mem(0x62, SET_SENSOR_ALTITUDE, buf, addrsize=16)

    # Store altitude in EEPROM
    persist_settings(i2c)


def get_sensor_altitude(i2c: SoftI2C) -> None:
    """
    Get the set sensor altitude. The altitude represenets meters above sea level.
    NOTE: Sensor must be in IDLE mode
    """
    if isIdle == False:
        print("# Error: Sensor is not IDLE")
        return

    buf: bytearray = bytearray(3)
    i2c.readfrom_mem_into(0x62, GET_SENSOR_ALTITUDE, buf, addrsize=16)
    if crc8(buf[0:2]) != buf[2]:
        print(
            "Received CRC %s did not match calculated CRC %s"
            % (hex(buf[2]), hex(crc8(buf[0:2])))
        )

    altitude = buf[0] << 8 | buf[1]
    print("Altitude is %s m" % altitude)


def set_ambient_pressure(i2c: SoftI2C, pressure: int) -> None:
    """
    The set_ambient_pressure command can be sent during periodic measurements to enable continuous pressure
    compensation. Note that setting an ambient pressure using set_ambient_pressure overrides any pressure compensation based
    on a previously set sensor altitude. Use of this command is highly recommended for applications experiencing significant ambient
    pressure changes to ensure sensor accuracy
    """
    if isIdle == False:
        print("# Error: Sensor is not IDLE")
        return

    buf: bytearray = bytearray(3)
    pressure = int(pressure / 100) # Signal conversion
    pressure_bytes = pressure.to_bytes(2, "big")
    buf[0] = pressure_bytes[0]  # MSB
    buf[1] = pressure_bytes[1]  # LSB
    buf[2] = crc8(buf[0:2])  # CRC
    i2c.writeto_mem(0x62, SET_AMBIENT_PRESSURE, b"", buf, addrsize=16)

    # Store pressure in EEPROM
    persist_settings(i2c)


def persist_settings(i2c: SoftI2C) -> None:
    i2c.writeto_mem(0x62, PERSIST_SETTINGS, b"", addrsize=16)
