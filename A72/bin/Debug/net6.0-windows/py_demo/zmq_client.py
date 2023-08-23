import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

socket.send(b"SPEC:CMD::Open")
message = socket.recv()
print(f"Received reply [ {message} ]")

#  Do 10 requests, waiting each time for a response
# for request in range(10):
#     print(f"Sending request {request} …")
#     socket.send(b"Hello")
#
#     #  Get the reply.
#     message = socket.recv()
#     print(f"Received reply {request} [ {message} ]")


# def OpenJeti():
#     socket.send("SPEC:CMD::Open")
#     message = socket.recv()
#     print(f"Received reply [ {message} ]")
#     pass


