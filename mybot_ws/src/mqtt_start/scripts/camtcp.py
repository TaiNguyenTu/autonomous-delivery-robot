#!/usr/bin/env python

import cv2
import socket
import struct
import rospy

SERVER_IP = "10.128.81.11"
SERVER_PORT = 6000

def send_image(sock, buf, nLen, imgeType):
    nImageType = struct.pack("!i", imgeType)
    nLen = struct.pack("!i", nLen)
    msgPack = nImageType + nLen + buf
    nRes = sock.send(msgPack)
    return nRes

def just_send_image(sock, img_buf, nlen):
    nlen = struct.pack("!i", nlen)
    try:
        nRes = sock.send(img_buf)
        print(len(img_buf))
        return nRes
    except:
        return -1

def init_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_addr = (SERVER_IP, SERVER_PORT)
    while True:
        try:
            sock.connect(server_addr)
            break
        except:
            print("Server is not open!!!!")
    return sock

def main():
    rospy.init_node('camtcp_node', anonymous=True)

    sock = init_socket()

    cap = cv2.VideoCapture("/dev/video0")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Capture failure")
        return -1


    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # send_image(sock, frame.tobytes(), frame.size, 1)
        result = just_send_image(sock, frame.tobytes(), frame.size)
        if result == -1:
            print("Reconnecting to server...")
            sock.close()
            sock = init_socket()        # print(len(frame.tobytes()))
        # cv2.imshow("Original", frame)
        # if cv2.waitKey(10) & 0xFF == 27:
        #     break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
