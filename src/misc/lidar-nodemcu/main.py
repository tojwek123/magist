from machine import Pin
from machine import I2C
from machine import UART
import uos
import time
import network
import random
from lidarlite import LidarLite
from sweep import Sweep, ProtocolError, SweepError, ChecksumError
import socket

LIDAR_LITE_UDP_PORT = 6969
SWEEP_UDP_PORT = 6868

class NonBlockingTask:
    
    def start(self):
        pass
        
    def stop(self):
        pass
        
    def process(self):
        pass

class LidarLiteTask(NonBlockingTask):
    
    def __init__(self, addr, port, lidar_lite):
        self._addr = addr
        self._port = port
        self._lidar_lite = lidar_lite
        self._udp_socket = None
        self._last_ticks_ms = 0
    
    def start(self):
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def stop(self):
        self._udp_socket = None
        
    def process(self, ticks_ms):
        if ticks_ms - self._last_ticks_ms > 1000:
            self._last_ticks_ms = ticks_ms
            distance = None                
            try:
                distance = self._lidar_lite.measure()
            except Exception as e:
                print('Failed to read from lidar, reason: {}'.format(e))
            if distance is not None:
                #print('Distance: {} cm'.format(distance))
                udp_buffer = (distance).to_bytes(2, 'big')
                try:
                    self._udp_socket.sendto(udp_buffer, (self._addr, self._port))
                except OSError:
                    pass
                    
class SweepTask(NonBlockingTask):
    
    STATE_INIT = 0
    STATE_READ = 1
    
    def __init__(self, addr, port, sweep):
        self._addr = addr
        self._port = port
        self._sweep = sweep
        self._udp_socket = None
        self._last_ticks_ms = 0
        self._state = SweepTask.STATE_INIT
        self._sweep.set_enabled(True)
        
    def start(self):
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def stop(self):
        self._udp_socket = None
        
    def process(self, ticks_ms):
        if SweepTask.STATE_INIT == self._state:
            if ticks_ms - self._last_ticks_ms > 1000:
                self._last_ticks_ms = ticks_ms
                try:
                    if self._sweep.is_ready():
                        print('Sweep ready')
                        self._sweep.start()
                        self._state = SweepTask.STATE_READ
                    else:
                        print('Sweep not ready')
                except ProtocolError as e:
                    print('Error during initialization: {}'.format(e))
        elif SweepTask.STATE_READ == self._state:
            try:
                for i in range(1000):
                    sample = self._sweep.read_sample()
                #print('Sample: {}'.format(sample))
            except ProtocolError as e:
                print('Error during reading: {}'.format(e))
                self._state = SweepTask.STATE_INIT
                self._sweep.set_enabled(0)
                time.sleep(0.1)
                self._sweep.set_enabled(1)
            except ChecksumError as e:
                print('Checksum error: {}'.format(e))
        
class App_State:
    DISCONNECTED = 1
    CONNECTING = 2
    GOT_CONNECTION = 3
    CONNECTED = 4
    
def main():
    #time.sleep(3)
    
    uart = UART(0)
    uart.init(115200, bits=8, parity=None, stop=1)
    #Disable REPL on uart
    uos.dupterm(None, 1)
    
    sweep = Sweep(uart, Pin(16, Pin.OUT))
    sweep_task = SweepTask('255.255.255.255', SWEEP_UDP_PORT, sweep)
    
    lidar_lite = LidarLite(I2C(scl=Pin(5), sda=Pin(4), freq=400000))
    lidar_lite_task = LidarLiteTask('255.255.255.255', LIDAR_LITE_UDP_PORT, lidar_lite)
    
    network_tasks = (sweep_task,)
    
    led_pin = Pin(2, Pin.OUT)
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    
    if sta_if.isconnected():
        app_state = App_State.GOT_CONNECTION
    else:
        app_state = App_State.DISCONNECTED
    
    last_ticks_ms = 0
    
    while True:    
        ticks_ms = time.ticks_ms()
        
        if App_State.DISCONNECTED == app_state:
            # sta_if.connect('HUAWEI-P20', '123456abc')
            sta_if.connect('UPCB596891', 'weptxpadrTf7')
            print('Connecting to the AP', end='')
            app_state = App_State.CONNECTING
        elif App_State.CONNECTING == app_state:
            if sta_if.isconnected():
                app_state = App_State.GOT_CONNECTION
            else:
                print('.', end='')
                time.sleep(0.1)
        elif App_State.GOT_CONNECTION == app_state:   
            print('Connected')
            print('Network config: {}'.format(sta_if.ifconfig()))
            
            for task in network_tasks:
                task.start()
                
            app_state = App_State.CONNECTED
        elif App_State.CONNECTED == app_state:
            if sta_if.isconnected():          
                for task in network_tasks:
                    task.process(ticks_ms)
                
                if ticks_ms - last_ticks_ms > 100:
                    last_ticks_ms = ticks_ms
                    led_pin.value(not led_pin.value())
            else:
                print('Connection with AP lost')
                app_state = App_State.DISCONNECTED
                
                for task in network_tasks:
                    task.stop()
        
        
if __name__ == '__main__':
    main()