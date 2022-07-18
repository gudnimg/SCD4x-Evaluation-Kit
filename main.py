from machine import SoftI2C
import SCD41

# create software I2C object
i2c : SoftI2C = SoftI2C(scl='X9', sda='X10', freq=400000)

# returns list of peripheral addresses
i2cAddresses : list = i2c.scan()

# SCD41 sensor should have I2C address 0x62
print("List of I2C addresses: %s" % [hex(addr) for addr in i2cAddresses])

# Ensure Sensor starts in IDLE mode at boot-up
SCD41.stop_periodic_measurement(i2c)

# Set current altitude
print("Setting Altitude to 41m")
SCD41.set_sensor_altitude(i2c, 41)

while(True):
    SCD41.stop_periodic_measurement(i2c)
    SCD41.get_sensor_altitude(i2c)
    SCD41.read_measurement(i2c)