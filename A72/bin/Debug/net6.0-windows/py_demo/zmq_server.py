import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    #  Wait for next request from client
    message = socket.recv_string()
    print(f"Received request: {message}")

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    str="OK"
    socket.send(str.encode('utf-8'))