# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import pandas as pd
import numpy as np
from queue import Queue
from threading import Thread
import math
from numpy import random
import time




# def oneToOne(argument):
#     # v = q1to1.get()
#
#
# def oneToOneStart(argument):
#     # arm.setjointangles(joint = [02873490381],speed = [982371])
#
# def sequencer(argument):
#
# def startSequencer(argument):
def findAngles(origin, newpoint):
    angle = math.atan2(newpoint[1] - origin[1], newpoint[0] - origin[0])
    return angle


def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance


def anglesOfBot(origin, otherbots):
    list = []
    dist = []
    for newpoint in otherbots:
        list.append(findAngles(origin, newpoint))
        dist.append(findDistance(origin, newpoint))
    return list, dist


def smoothAppend(array, x):
    check = array[-1]
    print(array)
    if abs(x - check) > 2:
        x = check + 2 * np.sign(x - check)
    array.append(x)
    print(array)
    return array


def movingAverage(array, numtoadd, window):
    if len(array) >= (window+1):
        windowArray = array[-window:-1]
        windowArray.append(numtoadd)
        avg = sum(windowArray)/len(windowArray)
    else:
        avg = numtoadd
    return avg




def MappingtoUse(argument):
    switcher = {
        0: [0, 0],
        1: [],
        2: "two",
    }

    # get() method of dictionary data type returns
    # value of passed argument if it is present
    # in dictionary otherwise second argument will
    # be assigned as default value of passed argument
    return switcher.get(argument, "nothing")


def Dance_copy(position,swap):
    playback = [0]
    pbsmooth = []
    while swap.empty():
        print(pbsmooth)
        dancemap = position.get()
        playback.append(dancemap)
        numtoadd = movingAverage(playback, dancemap, 3)
        pbsmooth.append(numtoadd)

    for i in range(len(pbsmooth)):
        print("playing")
        time.sleep(1)
        if i>2:
            input("enter to stio")
            break






# Driver program
if __name__ == "__main__":
    # x=random.randint(100, size=(200))
    # print(x)
    # start = time.time
    # r = pd.DataFrame()
    r=[]
    # RESP = pd.DataFrame(RESP)
    angle = []
    distance = []
    robots = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    coordinates = np.array(
        [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [1.0, 1.0], [3.0, 1.0], [5.0, 1.0], [2.0, 2.0], [3.0, 3.0],
         [4.0, 2.0]])
    for i in range(len(coordinates)):
        otherbots = np.delete(coordinates, i, 0)
        [list, dist] = anglesOfBot(coordinates[i], otherbots)
        angle.append(list)
        distance.append(dist)
    print(coordinates)
    # print({idx: {"coord": i, "angle": angle[idx]} for idx, i in enumerate(coordinates)})
    a = {idx: {"coord": i, "angle": angle[idx], "Distance": distance[idx], "name": robots[idx]} for idx, i in
         enumerate(coordinates)}
    print(a[2]["angle"][5])

    # x = int(input("next number"))
    # r.append(x)
    # print(r)
    q = Queue()
    q1 = Queue()
    Dcopy = Thread(target=Dance_copy, args=(q,q1))
    Dcopy.start()
    t = 0
    while True:
        x = int(input("next number"))
        # r = smoothAppend(r,x)
        # print(r)
        q.put(x)
        t += 1
        # print(t)
        if t > 5:
            q1.put(1)
            time.sleep(2)
            print("donje")
            break





