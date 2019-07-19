import socket
import select
from collections import deque

class AsyncTcpServer:
       
    def __init__(self, on_client_connected, on_client_disconnected, on_received_from_client, recv_buffer_size=128):
        self._server_sock = None
        self._client_sock = None
        self._on_client_connected = on_client_connected
        self._on_client_disconnected = on_client_disconnected
        self._on_received_from_client = on_received_from_client
        self._is_listening = False
        
    def _close_client_sock(self, client_sock):
        self._client_sock.close()
        
        if self._on_client_disconnected is not None:
            self._on_client_disconnected(self._client_sock)
        
        self._client_sock = None
        
    def start_listening(self, addr, port):
        if self._is_listening:
            stop_listening()
        
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.settimeout(0.01)
        self._server_sock.bind((addr, port))
        self._server_sock.listen(1)
        self._is_listening = True
        
    def stop_listening(self):
        if self._is_listening:
            if self._client_sock is not None:
                self._close_client_sock(client_sock)
            self._server_sock.close()
            self._is_listening = False        
            
    def write(self, sock, data):
        try:
            sock.settimeout(1)
            sock.write(data)
            sock.settimeout(0.01)
        except OSError:
            self._close_client_sock(sock)
        
    def process(self):
        if self._is_listening:
            if self._client_sock is None:
                try:
                    self._client_sock, addr = self._server_sock.accept()
                except OSError:
                    pass
                    
                if self._client_sock is not None:
                    if self._on_client_connected is not None:
                        self._on_client_connected(self._client_sock, addr)
            else:
                try:
                    recv_data = self._client_sock.recv(16)
                except OSError:
                    return
                
                if recv_data is None or len(recv_data) == 0:
                    self._close_client_sock(self._client_sock)
                else:
                    if self._on_received_from_client is not None:
                        self._on_received_from_client(self._client_sock, recv_data)
                    