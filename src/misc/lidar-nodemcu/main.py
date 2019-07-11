from machine import Pin
from machine import I2C
import time
import network
import socket
import random

class Lidar:
    
    I2C_ADDRESS = 0x62
    REG_CMD = 0x00
    REG_STATUS = 0x01
    REG_DIST_MSB = 0x0F
    REG_DIST_LSB = 0x10
    CMD_GET_DIST = 0x04
    STATUS_BUSY_MASK = 0b00000001
    
    def __init__(self):
        self._i2c = I2C(scl=Pin(5), sda=Pin(4), freq=400000)
    
    def write_reg(self, reg, data):
        buff = bytes([reg, data])
        self._i2c.writeto(Lidar.I2C_ADDRESS, buff)
        
    def read_reg(self, reg):
        buff = bytes([reg])
        self._i2c.writeto(Lidar.I2C_ADDRESS, buff)
        return self._i2c.readfrom(Lidar.I2C_ADDRESS, buff)[0]
    
    def measure(self):
        # self.write_reg(Lidar.REG_CMD, Lidar.CMD_GET_DIST)
        # status = 0#self.read_reg(Lidar.REG_STATUS)
        
        # while (status & Lidar.STATUS_BUSY_MASK) != 0:
            # status != self.read_reg(Lidar.REG_STATUS)
            
        # dist_cm = self.read_reg(Lidar.REG_DIST_MSB) << 8
        # dist_cm += self.read_reg(Lidar.REG_DIST_LSB)
        
        dist_cm = random.getrandbits(16)
        
        return dist_cm

class App_State:
    DISCONNECTED = 1
    CONNECTING = 2
    GOT_CONNECTION = 3
    CONNECTED = 4    

def main():
    time.sleep(1)

    print('Running...')

    led_pin = Pin(2, Pin.OUT)
    lidar = Lidar()
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    
    if sta_if.isconnected():
        app_state = App_State.GOT_CONNECTION
    else:
        app_state = App_State.DISCONNECTED
    
    while True:    
        if App_State.DISCONNECTED == app_state:
            sta_if.connect('UPCB596891', 'weptxpadrTf7')
            print('Connecting to the AP', end='')
            app_state = App_State.CONNECTING
        elif App_State.CONNECTING == app_state:
            if sta_if.isconnected():
                app_state = App_State.GOT_CONNECTION
            else:
                print('.', end='')
                time.sleep(0.5)
        elif App_State.GOT_CONNECTION == app_state:   
            print('Connected')
            print('Network config: {}'.format(sta_if.ifconfig()))
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            app_state = App_State.CONNECTED
        elif App_State.CONNECTED == app_state:
            if sta_if.isconnected():
                distance = None                
                try:
                    distance = lidar.measure()
                except Exception as e:
                    print('Failed to read from lidar, reason: {}'.format(e))
                    
                if distance is not None:
                    print('Distance: {} cm'.format(distance))
                    udp_buffer = (distance).to_bytes(2, 'big')
                    udp_socket.sendto(udp_buffer, ('255.255.255.255', 6969))
                led_pin.value(not led_pin.value())
                time.sleep(0.1)
            else:
                print('Connection with AP lost')
                app_state = App_State.DISCONNECTED
        
        
if __name__ == '__main__':
    main()