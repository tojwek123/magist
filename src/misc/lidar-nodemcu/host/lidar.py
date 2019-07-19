import socket
import threading
import time

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
        s.bind(('', 6969))
        s.settimeout(1)
        
        while self._running:
            try:
                (data, addr) = s.recvfrom(1024)
                distance = int.from_bytes(data, byteorder='big', signed=False)
                
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
        
    def read_now(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('', 6969))
        s.settimeout(1)
        (data, addr) = s.recvfrom(1024)
        distance = int.from_bytes(data, byteorder='big', signed=False)
        return distance
        

def on_lidar_reading(distance):
    print(distance)
    
def on_lidar_error(error):
    print('Error: {}'.format(error))

def main():
    lidar = Lidar(on_lidar_reading, on_lidar_error)
    lidar.run()
   
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        lidar.stop()

if __name__ == '__main__':
    main()