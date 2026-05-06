import socket

HOST = "0.0.0.0"
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print("Listening on {}:{}".format(HOST, PORT))

try:
    while True:
        conn, addr = server.accept()
        print("Connected: {}".format(addr))
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    print("Disconnected: {}".format(addr))
                    break
                print("RX: {}".format(data.decode("utf-8", errors="replace").strip()))
                conn.sendall(data)
        finally:
            conn.close()
finally:
    server.close()