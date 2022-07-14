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


def start_periodic_measurement(i2c: SoftI2C) -> None:
    """
    Start periodic measurement, signal update interval is 5 seconds.
    """
    i2c.writeto_mem(0x62, START_PERIODIC_MEASUREMENT, b"", addrsize=16)


def stop_periodic_measurement(i2c: SoftI2C) -> None:
    """
    Stop periodic measurement to change the sensor configuration or to save power.
    Note that the sensor will only respond to other commands after waiting 500 ms
    after issuing the stop_periodic_measurement command.
    """
    i2c.writeto_mem(0x62, STOP_PERIODIC_MEASUREMENT, b"", addrsize=16)
    time.sleep(0.5)  # Sensor becomes responsive after 500ms


def is_data_ready(i2c: SoftI2C) -> bool:
    buf: bytearray = bytearray(3)
    i2c.readfrom_mem_into(0x62, GET_DATA_READY_STATUS, buf, addrsize=16)
    return ((buf[0] & 0x7) << 8) | buf[1] != 0


def read_measurement(i2c: SoftI2C) -> None:

    buf: bytearray = bytearray(9)

    print("# STOP_PERIODIC_MEASUREMENT")
    stop_periodic_measurement(i2c)

    print("# START_PERIODIC_MEASUREMENT")
    start_periodic_measurement(i2c)

    print("# GET_DATA_READY_STATUS")
    while is_data_ready(i2c) == False:
        continue

    print("# READ_MEASUREMENT")
    i2c.readfrom_mem_into(0x62, READ_MEASUREMENT, buf, addrsize=16)

    #TODO: check CRC byte buf[2]
    CO2_ppm = buf[0] << 8 | buf[1]
    print("CO2: %sppm" % CO2_ppm)

    #TODO: check CRC byte buf[5]
    temperature = buf[3] << 8 | buf[4]
    temperature = -45 + (175 * temperature) / (2**16 - 1)
    print("Temperature: %.2f°C" % temperature)

    #TODO: check CRC byte buf[8]
    relative_humidity = buf[6] << 8 | buf[7]
    relative_humidity = (100 * relative_humidity) / (2**16 - 1)
    print("Humidity: %.2f%%" % relative_humidity)
