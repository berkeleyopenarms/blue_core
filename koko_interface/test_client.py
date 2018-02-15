#!usr/bin/env python

import json
import time
import socket
import pprint

print("Testing ROSBridge Client")

TCP_IP = 'hekate.cs.berkeley.edu'
# TCP_IP = ''
TCP_PORT = 9090
BUFFER_SIZE = 4096

def encode_message(msg):
    return (json.dumps(msg)).encode()

def decode_message(msg):
    return json.loads(msg.decode())

advertise_msg = {
    "op": "advertise", 
    "topic": "/test_client", 
    "type": "std_msgs/String"
}


my_message = {
    "data" : "Hello World"
}

pub_msg = {
    "op": "publish", 
    "topic": "/test_client", 
    "msg": my_message
}

call_service_params = {
    "a": 23,
    "b": 32
}

call_service_msg = {
    "op": "call_service", 
    "service": "/add_two_ints", 
    "args": call_service_params
}

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sub_msg = {
    "op": "subscribe",
    "topic": "/rosout"
}

s.connect((TCP_IP, TCP_PORT))

print("Sending JSON data")
s.send(encode_message(advertise_msg))
time.sleep(0.1)
s.send(encode_message(pub_msg))
time.sleep(0.1)
s.send(encode_message(call_service_msg))
s.send(encode_message(sub_msg))

while True:
    data = s.recv(BUFFER_SIZE)
    print("Received JSON response: ")
    print(data)
    pprint.pprint(decode_message(data))
    time.sleep(1)

s.close()
