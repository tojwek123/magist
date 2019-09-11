import socket
import threading
import time
import struct

class Lidar:
    
    UDP_PORT = 6969
    ERROR_TIMEOUT = -1
    
    def __init__(self, on_reading=None, on_error=None):
        self._running = False
        self._thread = False
        self._on_reading = on_reading
        self._on_error = on_error
        
    def _task(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('', Lidar.UDP_PORT))
        s.settimeout(1)
        
        while self._running:
            try:
                (data, addr) = s.recvfrom(1024)
                (distance, ) = struct.unpack('>H', data)
                
                if self._on_reading is not None:
                    self._on_reading(distance)
            except socket.timeout:
                if self._on_error is not None:
                    self._on_error(Lidar.ERROR_TIMEOUT)
                
    def run(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._task)
            self._thread.start()
    
    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()
            self._thread = None
        
class Sweep:
    
    UDP_PORT = 6868
    ERROR_TIMEOUT = -1
    
    def __init__(self, on_reading=None, on_error=None):
        self._running = False
        self._thread = False
        self._on_reading = on_reading
        self._on_error = on_error
        
    def _task(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('', Sweep.UDP_PORT))
        s.settimeout(2)
        
        while self._running:
            try:
                (data, addr) = s.recvfrom(32 * 1024)
                scan = []
                for i in range(len(data) / 7):
                    sample = struct.unpack('>LHB', data[:7])
                    scan.append(sample)
                    data = data[7:]
 
                if self._on_reading is not None:
                    self._on_reading(scan)
            except socket.timeout:
                if self._on_error is not None:
                    self._on_error(Sweep.ERROR_TIMEOUT)
                
    def run(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._task)
            self._thread.start()
    
    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()
            self._thread = None

def main():
    def on_lidar_reading(distance):
        print('lidar', distance)
    
    def on_lidar_error(error):
        print('Error: {}'.format(error))
        
    def on_sweep_reading(angle, distance, signal_strength):
        print('sweep', angle, distance, signal_strength)
        
    def on_sweep_error(error):
        print('Error: {}'.format(error))


    lidar = Lidar(on_lidar_reading, on_lidar_error)
    sweep = Sweep(on_sweep_reading, on_sweep_error)
    
    lidar.run()
    sweep.run()
   
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        lidar.stop()
        sweep.stop()
        
if __name__ == '__main__':
    main()
