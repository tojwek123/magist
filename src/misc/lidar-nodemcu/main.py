from machine import Pin
from machine import I2C
import time
import network
import random
from asynctcpserver import AsyncTcpServer
from lidarlite import LidarLite
import socket

UART_TUNNEL_TCP_PORT = 6868
LIDAR_LITE_PUBLISHER_PORT = 6969

class NonBlockingTask:
    
    def start(self):
        pass
        
    def stop(self):
        pass
        
    def process(self):
        pass

class UartTunnel(NonBlockingTask):
    
    def __init__(self, addr, port):
        self._addr = addr
        self._port = port
        self._tcp_server = AsyncTcpServer(self._on_tcp_client_connected, self._on_client_disconnected, self._on_received_from_client)
        self._last_ticks_ms = 0 
        self._client_sock = None
        self._test = 0
        
    def start(self):
        self._tcp_server.start_listening(self._addr, self._port)
        
    def stop(self):
        self._tcp_server.stop_listening()
        
    def process(self, ticks_ms):
        self._tcp_server.process()
        
        if ticks_ms - self._last_ticks_ms > 100:
            self._last_ticks_ms = ticks_ms
            if self._client_sock is not None:
                self._test += 1
                buff = '{}\r\n'.format(self._test).encode()
                self._tcp_server.write(self._client_sock, buff)
                time.sleep(0.1)
    
    def _on_tcp_client_connected(self, client_sock, addr):
        print('New client connected, addr: {}'.format(addr))
        self._client_sock = client_sock

    def _on_client_disconnected(self, client_sock):
        print('Client disconnected')
        self._client_sock = None
        
    def _on_received_from_client(self, client_sock, recv_data):
        print('Received from client, data: {}'.format(recv_data))

class LidarLitePublisher(NonBlockingTask):
    
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

class App_State:
    DISCONNECTED = 1
    CONNECTING = 2
    GOT_CONNECTION = 3
    CONNECTED = 4
    
def main():
    time.sleep(1)
    print('Running...')
    
    lidar_lite = LidarLite(I2C(scl=Pin(5), sda=Pin(4), freq=400000))
    lidar_lite_publisher = LidarLitePublisher('255.255.255.255', LIDAR_LITE_PUBLISHER_PORT, lidar_lite)
    uart_tunnel = UartTunnel('0.0.0.0', UART_TUNNEL_TCP_PORT)
    
    network_tasks = (lidar_lite_publisher, uart_tunnel)
    
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
            sta_if.connect('HUAWEI-P20', '123456abc')
            print('Connecting to the AP', end='')
            app_state = App_State.CONNECTING
        elif App_State.CONNECTING == app_state:
            if sta_if.isconnected():
                app_state = App_State.GOT_CONNECTION
            else:
                print('.', end='')
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