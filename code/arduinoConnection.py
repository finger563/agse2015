#!/usr/bin/python

import time
import serial
import random

serialPort = '/dev/ttyTHS0'
baudRate = 9600
loopIterations = 100
messageDelay = 1

makeRandom = True

msgs = [
    'READ\n',
    'WRITE:128,128,0\n'
]

def removeSentFromReceived(sentList,recvStr):
    recv = ''
    inRecvStr = False
    for i in range(0,len(sentList)):
        if int(sentList[i]) == ord(recvStr[i]):
            inRecvStr = True
        else:
            inRecvStr = False
            break
    if inRecvStr == True:
        recv = recvStr[len(sentList):]
    return recv

if __name__=="__main__":
    ser = serial.Serial(port=serialPort,
                        baudrate=baudRate,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE)
    ser.isOpen()
    random.seed()
    angle = 1000
    #for i in range(loopIterations):
    while True:
        #msgs[1] = "WRITE:{},{},{}\n".format(random.randrange(255),random.randrange(255),random.randrange(90))
        angle = -angle
        msgs[1] = "WRITE:{},{},{}\n".format(128,250,-angle)
        for msg in msgs:
            for char in msg:
                ser.write(char)
            out = ''
            time.sleep(messageDelay)
            while ser.inWaiting() > 0:
                out += ser.read()
            if out != '':
                #recv = removeSentFromReceived(msg,out)
                print out
    ser.close()
