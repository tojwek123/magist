import socket
import threading
import time
import struct
from sweeppy import Sweep
from lidarlite import LidarLite

LIDARLITE_UDP_PORT = 6969
SWEEP_UDP_PORT = 6868

def lidarlite_publisher():
    lidar = LidarLite()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while True:
        try:
            distance_cm = lidar.measure()
        except OSError:
            print('Failed to read from Lidar Lite')
        else:
            #print('Lidar distance: {} cm'.format(distance_cm))
            buffer = struct.pack('>H', distance_cm)
            sock.sendto(buffer, ('255.255.255.255', LIDARLITE_UDP_PORT)) 
        time.sleep(0.05)

def sweep_publisher():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    with Sweep('/dev/serial0') as sweep:
        sweep.start_scanning()
        
        for scans in sweep.get_scans():
            for scan in scans:
                for sample in scan:
                    #print('angle: {:03f} deg, distance: {} deg'.format(sample.angle/1000, sample.distance))
                    buffer = struct.pack('>LHB', sample.angle, sample.distance, sample.signal_strength)
                    sock.sendto(buffer, ('255.255.255.255', SWEEP_UDP_PORT))
                    
                    #Let the lidarlite thread run
                    time.sleep(0.0001)

    print('dupa')

def main():
    lidarlite_thread = threading.Thread(target=lidarlite_publisher)
    sweep_thread = threading.Thread(target=sweep_publisher)
    
    lidarlite_thread.start()
    sweep_thread.start()

    lidarlite_thread.join()
    sweep_thread.join()

    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    #while True:
     #   sock.sendto(b'elo', ('255.255.255.255', 6969))
      #  time.sleep(1)

if __name__ == '__main__':
    main()
