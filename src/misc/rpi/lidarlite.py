import smbus

SMBUS_NO = 1 #/dev/i2c-1
I2C_ADDR = 0x62

REG_CMD = 0x00
REG_STATUS = 0x01
REG_DIST_MSB = 0x0F
REG_DIST_LSB = 0x10

CMD_GET_DIST = 0x04
STATUS_BUSY_MASK = 0b00000001

class LidarLite:
    def __init__(self):
        self._bus = smbus.SMBus(1)

    def write_reg(self, reg, data):
        self._bus.write_byte_data(I2C_ADDR, reg, data)

    def read_reg(self, reg):
        self._bus.write_byte(I2C_ADDR, reg)
        return self._bus.read_byte(I2C_ADDR)

    def measure(self):
        self.write_reg(REG_CMD, CMD_GET_DIST)
        status = self.read_reg(REG_STATUS)

        while (status & STATUS_BUSY_MASK) != 0:
            status = self.read_reg(REG_STATUS)

        distance_cm = self.read_reg(REG_DIST_MSB) << 8
        distance_cm += self.read_reg(REG_DIST_LSB)

        return distance_cm

import time

def main():
    lidar = LidarLite()

    while True:
        distance_cm = lidar.measure()
        print('Distance: {} cm'.format(distance_cm))
        time.sleep(0.05)

if __name__ == '__main__':
    main()


