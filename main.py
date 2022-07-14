from machine import SoftI2C

# create software I2C object
i2c = SoftI2C(scl='X9', sda='X10', freq=400000)

# returns list of peripheral addresses
i2cAddresses : list = i2c.scan()

# SCD41 sensor should have I2C address 0x62
print("List of I2C addresses: %s" % [hex(addr) for addr in i2cAddresses])