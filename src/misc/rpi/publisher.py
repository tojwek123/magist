import socket
import threading
import time
import struct
import sys
from sweeppy import Sweep
from lidarlite import LidarLite

LIDARLITE_UDP_PORT = 6969
SWEEP_UDP_PORT = 6868

SWEEP_START_ANGLE_DEG = 160
SWEEP_END_ANGLE_DEG = 200

VERBOSE = False

def lidarlite_publisher():
    global VERBOSE

    lidar = LidarLite()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while True:
        try:
            distance_cm = lidar.measure()
        except OSError:
            print('Failed to read from Lidar Lite')
        else:
            if VERBOSE:
                print('Lidar distance: {} cm'.format(distance_cm))
            buffer = struct.pack('>H', distance_cm)
            sock.sendto(buffer, ('255.255.255.255', LIDARLITE_UDP_PORT)) 
        time.sleep(0.05)

def sweep_publisher():
    global VERBOSE

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    with Sweep('/dev/serial0') as sweep:
        sweep.start_scanning()
        
        for scans in sweep.get_scans():
            for scan in scans:
                buffer = b''
                for sample in scan:
                    if sample.angle/1000 > SWEEP_START_ANGLE_DEG and sample.angle/1000 < SWEEP_END_ANGLE_DEG:
                        if VERBOSE:
                            print('angle: {:03f} deg, distance: {} cm'.format(sample.angle/1000, sample.distance))
                        buffer += struct.pack('>LHB', sample.angle, sample.distance, sample.signal_strength)
                        time.sleep(0.00001)
                sock.sendto(buffer, ('255.255.255.255', SWEEP_UDP_PORT))

    print('dupa')

def main():
    global VERBOSE

    if len(sys.argv) > 1:
        if '-v' == sys.argv[1]:
            VERBOSE = True

    lidarlite_thread = threading.Thread(target=lidarlite_publisher)
    sweep_thread = threading.Thread(target=sweep_publisher)
    
    lidarlite_thread.start()
    sweep_thread.start()

    lidarlite_thread.join()
    sweep_thread.join()

if __name__ == '__main__':
    main()
