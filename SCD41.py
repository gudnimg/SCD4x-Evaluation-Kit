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


def is_data_ready(i2c: SoftI2C) -> bool:
    buf: bytearray = bytearray(3)
    i2c.readfrom_mem_into(0x62, GET_DATA_READY_STATUS, buf, addrsize=16)
    return ((buf[0] & 0x7) << 8) | buf[1] != 0


def read_measurement(i2c: SoftI2C) -> None:

    buf: bytearray = bytearray(9)

    print("# STOP_PERIODIC_MEASUREMENT")
    i2c.writeto_mem(0x62, STOP_PERIODIC_MEASUREMENT, b"", addrsize=16)
    time.sleep(1)  # Sensor becomes responsive after 500ms
    print("# START_PERIODIC_MEASUREMENT")
    i2c.writeto_mem(0x62, START_PERIODIC_MEASUREMENT, b"", addrsize=16)

    # Wait for measurement to be ready
    print("# GET_DATA_READY_STATUS")
    while is_data_ready(i2c) == False:
        continue

    # Read the measurement
    print("# READ_MEASUREMENT")
    i2c.readfrom_mem_into(0x62, READ_MEASUREMENT, buf, addrsize=16)

    CO2_ppm = buf[0] << 8 | buf[1]
    print("CO2 (ppm): %s" % CO2_ppm)
