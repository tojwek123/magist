import socket

def main():
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', 6969))
    
    while True:
        (data, addr) = s.recvfrom(1024)
        distance = int.from_bytes(data, byteorder='big', signed=False)
        print('Distance: {} cm'.format(distance))
    
if __name__ == '__main__':
    main()