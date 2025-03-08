import threading
import time
import serial
import numpy as np
import utils
from numpy.random import rand

ANCHOR_NUM = 4
TAG_NUM = 1

tagComs = ['COM14']
tagQueues = []

tagSerials = []

MAX_TIME = 90

PATH = '../实验数据/手写轨迹/'
FILE_NAME = 'handwriting_info3'

baudRate = 115200

for i in range(TAG_NUM):
    tagSerials.append (serial.Serial(tagComs[i], baudRate, 8, 'N', 1))
    tagSerials[i].set_buffer_size(rx_size=1024*1024*100)
    tagQueues.append(b'')

startTime = time.time()
lastTime = time.time()

for i in range(TAG_NUM):
    serialData = tagSerials[i].read_all() # Read all data from serial port

while (time.time()-startTime) < MAX_TIME:
    if (time.time()-lastTime > 2):
        print(time.time()-startTime)
        lastTime = time.time()

    for i in range(TAG_NUM):

        serialData = tagSerials[i].read_all() # Read all data from serial port
        tagQueues[i] += serialData

byteData = tagQueues[i]

print('Data acquisition finished!')
# print(tagQueues)

for i in range(TAG_NUM):

    file = open(PATH+FILE_NAME+'_byte'+str(i+1)+'.txt', 'wb')

    file.write(tagQueues[i])
    # while not tagQueues[i].empty():

    #     uwbData = tagQueues[i].get()
    #     file.write(uwbData)
    file.close()

utils.byte2txt(byteData=byteData, path=PATH, name=FILE_NAME, anchor_num=ANCHOR_NUM)
utils.seperate(path=PATH, name=FILE_NAME, tag_num=1, anchor_num=ANCHOR_NUM)