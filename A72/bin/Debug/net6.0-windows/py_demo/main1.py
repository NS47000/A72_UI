#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq
import numpy
import cv2

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.100.40.78:5555")

#  Do 10 requests, waiting each time for a response
for request in range(1):
    print(f"Sending request {request} …")
    socket.send(b"GetX")

    # row11=socket.recv_string(zmq.RCVMORE)
    #  Get the reply.
    # Receiving image in bytes
    rows = int(socket.recv_string())
    cols = int(socket.recv_string())
    channel = int(socket.recv_string())
    depth = int(socket.recv_string())
    image_bytes = socket.recv()
    imageY_bytes = socket.recv()
    imageZ_bytes = socket.recv()

    if(depth == cv2.CV_8U):
        dtype = numpy.uint8
    elif depth == cv2.CV_16U:
        dtype = numpy.uint16


    # Converting bytes data to ndarray
    image = numpy.frombuffer(image_bytes, dtype)
    image = image.reshape(rows, cols, channel)
    imageY = numpy.frombuffer(imageY_bytes, dtype)
    imageY = imageY.reshape(rows, cols, channel)
    imageZ = numpy.frombuffer(imageZ_bytes, dtype)
    imageZ = imageZ.reshape(rows, cols, channel)

    

    cv2.imwrite("C:/Users/tao.sun/Desktop/VVV-X.tif", image)
    cv2.imwrite("C:/Users/tao.sun/Desktop/VVV-Y.tif", imageY)
    cv2.imwrite("C:/Users/tao.sun/Desktop/VVV-Z.tif", imageZ)
    #cv2.imshow("ReadImg", image)

    print(f"Received reply {request} [ {image_bytes } ]")