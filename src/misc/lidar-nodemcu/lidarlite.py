from machine import I2C
import random

I2C_ADDRESS = 0x62
REG_CMD = 0x00
REG_STATUS = 0x01
REG_DIST_MSB = 0x0F
REG_DIST_LSB = 0x10
CMD_GET_DIST = 0x04
STATUS_BUSY_MASK = 0b00000001

class LidarLite:
    
    def __init__(self, i2c):
        self._i2c = i2c
    
    def write_reg(self, reg, data):
        buff = bytes([reg, data])
        self._i2c.writeto(I2C_ADDRESS, buff)
        
    def read_reg(self, reg):
        buff = bytes([reg])
        self._i2c.writeto(I2C_ADDRESS, buff)
        return self._i2c.readfrom(I2C_ADDRESS, buff)[0]
    
    def measure(self):
        # self.write_reg(REG_CMD, CMD_GET_DIST)
        # status = self.read_reg(REG_STATUS)
        
        # while (status & STATUS_BUSY_MASK) != 0:
            # status != self.read_reg(REG_STATUS)
            
        # dist_cm = self.read_reg(REG_DIST_MSB) << 8
        # dist_cm += self.read_reg(REG_DIST_LSB)
        
        dist_cm = random.getrandbits(16)
        
        return dist_cm