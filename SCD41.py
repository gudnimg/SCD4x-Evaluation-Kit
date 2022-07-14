import time
from machine import SoftI2C

# I2C commands
START_PERIODIC_MEASUREMENT = 0x21B1
STOP_PERIODIC_MEASUREMENT  = 0x3F86
READ_MEASUREMENT           = 0xEC05
GET_DATA_READY_STATUS      = 0xE4B8

def read_measurement(i2c: SoftI2C) -> None:
    
    buf : bytearray = bytearray(9)

    print("# STOP_PERIODIC_MEASUREMENT")
    i2c.writeto_mem(0x62, STOP_PERIODIC_MEASUREMENT, b'', addrsize=16)
    time.sleep(1)
    print("# START_PERIODIC_MEASUREMENT")
    i2c.writeto_mem(0x62, START_PERIODIC_MEASUREMENT, b'', addrsize=16)

    # Wait for measurement to be ready
    mystatus = True
    buf[0] = GET_DATA_READY_STATUS
    print("# GET_DATA_READY_STATUS")
    while(mystatus):
        i2c.readfrom_mem_into(0x62, GET_DATA_READY_STATUS, buf, addrsize = 16)
        time.sleep(0.001) # Sleep for 1ms
        if ((buf[0]& 0x7) << 8) | buf[1] != 0:
            mystatus = False

    # Read the measurement
    print("# READ_MEASUREMENT")
    i2c.readfrom_mem_into(0x62, READ_MEASUREMENT, buf, addrsize = 16)

    CO2_ppm = buf[0] << 8 | buf[1]
    print("CO2 (ppm): %s" % CO2_ppm)
