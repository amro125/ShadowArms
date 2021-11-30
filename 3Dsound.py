# This is a sample Python script.
import numpy as np
import queue
from threading import Thread
import time
from xarm.wrapper import XArmAPI
from pythonosc import udp_client
import socket
import pickle
import struct
from queue import Queue
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.



def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        # a.set_servo_angle(angle=[0.0, 0.0, 0.0, 1.57, 0.0, 0, 0.0], wait=True, speed=0.4, acceleration=0.25,
        #                   is_radian=True)
        a.set_position(*[121, -1, 585, -2, -86, -178], wait=False)


def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance





# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    arm1 = XArmAPI('192.168.1.208')
    arm2 = XArmAPI('192.168.1.244')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.236')
    arm5 = XArmAPI('192.168.1.226')
    arm6 = XArmAPI('192.168.1.242')
    arm7 = XArmAPI('192.168.1.215')
    arm8 = XArmAPI('192.168.1.234')
    arm9 = XArmAPI('192.168.1.237')
    arm10 = XArmAPI('192.168.1.204')

    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9, arm10]
    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()

    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    coordinates = np.array(
        [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [1.0, 1.0], [3.0, 1.0], [5.0, 1.0], [2.0, 2.0], [5.0, 3.0],
         [4.0, 2.0]])
    xValue = []
    yValue = []
    coordinateScale = []
    for c in coordinates:
        x = np.interp(c[0], (0, 6), (0, 127))
        y = np.interp(c[1], (0, 3), (0, 127))
        coordinateScale.append([x,y])

    distances = []
    for s in coordinateScale:
        d = findDistance(s, [0, 0])
        distances.append(d)
    lastdist = distances
    print(lastdist)


    # def changeDir():
    #     while True:
    #         angle = int(input("desired angle"))
    #         q.put(angle)
    def UDP():
        print("started!")
        x = [0, 0]
        xP = [0, 0]
        while True:
            IP = "192.168.1.1"
            # IP = "10.0.0.1"
            PORT_TO_MAX = 7980

            message = [2, 2]  # Message is [robot to make sound, track in playlist to play]
            # list can be anything
            sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            sock.bind((IP, PORT_TO_MAX))
            #sock.set t(None)
            sock.settimeout(None)
            data, server = sock.recvfrom(1024)
            received = []

            if data.startswith(b'int'):
                received = struct.unpack('!I', data[-4:])[0]
            elif data.startswith(b'list'):
                for a in range(len(x)):
                    if a == 0:
                        x[a+1] = int((repr(struct.unpack('!I', data[-4:])[0])))
                    else:
                        x[a-1] = int((repr(struct.unpack('!I', data[-(4 * (a + 1)):-(4 * a)])[0])))

                # x[0] = (repr(struct.unpack('!I', data[-4:])[0]))
                # print(repr(struct.unpack('!I', data[-8:-4])[0]))
                # print(repr(struct.unpack('!I', data[-12:-8])[0]))  # eight
                # print(repr(struct.unpack('!I', data[-16:-12])[0])) # seven
                # print(repr(struct.unpack('!I', data[-20:-16])[0]))  # six
                # print(repr(struct.unpack('!I', data[-24:-20])[0]))  # fifth
                # print(repr(struct.unpack('!I', data[-28:-24])[0]))  # fourth
                # print(repr(struct.unpack('!I', data[-32:-28])[0]))  # third
                # print(repr(struct.unpack('!I', data[-36:-32])[0]))  # second
                # print(repr(struct.unpack('!I', data[-40:-36])[0]))  # First
            else:
                received = data.decode('utf-8')

            # if (x - xP) != 0:
            #     xP = x
            #     print("switch", x)
            # if (x[1] - xP[1]) != 0:
            #     xP = x
            #     print("switch", x)


            #print(x, xP)
            #print("sent!")
            q0.put(x)


    def livetraj(inq, robot):
        tf = 2
        # q i
        # 0.2 * np.floor(xi / 0.2)
        # range is 200 to -200
        t0 = 0
        t = t0
        q_i = 0
        q_dot_i = 0
        q_dot_f = 0
        q_dotdot_i = 0
        q_dotdot_f = 0
        t_array = np.arange(0, tf, 0.006)
        p = 0
        v = 0
        a = 0
        while True:
            goal = inq.get()
            print("moving",robot)
            q_i = p
            q_dot_i = 0
            q_dotdot_i = 0
            q_f = goal
            i = 0

            while i <= len(t_array):
                start_time = time.time()
                if inq.empty() == False:
                    goal = inq.get()
                    print("switch bot", robot)
                    q_i = p
                    q_dot_i = v
                    q_dotdot_i = 0
                    q_f = goal
                    i = 0
                    # IF YOU WANT TO ADD SPEED CHANGES THEN SWAP THE ABOVE LINES WITH THE BELOW LINES
                    # # q should input an array of [*absolute* position of joint, time(in seconds) to reach there]
                    # q_f = goal[0]
                    # tf = goal[1]
                    # t_array = np.arange(0, tf, 0.006)
                    # print("switch")
                if i == len(t_array):
                    t = tf
                else:
                    t = t_array[i]
                a0 = q_i
                a1 = q_dot_i
                a2 = 0.5 * q_dotdot_i
                a3 = 1.0 / (2.0 * tf ** 3.0) * (20.0 * (q_f - q_i) - (8.0 * q_dot_f + 12.0 * q_dot_i) * tf - (
                        3.0 * q_dotdot_f - q_dotdot_i) * tf ** 2.0)
                a4 = 1.0 / (2.0 * tf ** 4.0) * (30.0 * (q_i - q_f) + (14.0 * q_dot_f + 16.0 * q_dot_i) * tf + (
                        3.0 * q_dotdot_f - 2.0 * q_dotdot_i) * tf ** 2.0)
                a5 = 1.0 / (2.0 * tf ** 5.0) * (12.0 * (q_f - q_i) - (6.0 * q_dot_f + 6.0 * q_dot_i) * tf - (
                        q_dotdot_f - q_dotdot_i) * tf ** 2.0)

                p = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
                v = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
                a = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
                mvpose = [121, -1, 585 + p, -2, -86, -178] #x y z roll pitch yaw
                arms[robot].set_servo_cartesian(mvpose, speed=100, mvacc=2000)
                # arm6.set_servo_angle_j(angles=[0, 0, 0, 90, 0, p, 0], is_radian=False)
                tts = time.time() - start_time
                sleep = 0.006 - tts

                if tts > 0.006:
                    sleep = 0

                # print(tts)
                time.sleep(sleep)
                i += 1
                # if t == 1:
                # print(t, p, v, a)
            print("done")


    q0 = queue.Queue()
    t = Thread(target=UDP)
    # t2 = Thread(target=changeDir)
    # t3 = Thread(target=livetraj(q))
    # #t2.start()
    # t3.start()
    t.start()
    i = 0

    q = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    q5 = Queue()
    q6 = Queue()
    q7 = Queue()
    q8 = Queue()
    q9 = Queue()

    quay = [q, q1, q2, q3, q4, q5, q6, q7, q8, q9]

    t2 = Thread(target=livetraj, args=(q,0,))
    t3 = Thread(target=livetraj, args=(q1,1,))
    t4 = Thread(target=livetraj, args=(q2,2,))
    t5 = Thread(target=livetraj, args=(q3,3,))
    t6 = Thread(target=livetraj, args=(q4,4,))
    t7 = Thread(target=livetraj, args=(q5,5,))
    t8 = Thread(target=livetraj, args=(q6,6,))
    t9 = Thread(target=livetraj, args=(q7,7,))
    t10 = Thread(target=livetraj, args=(q8,8,))
    t11 = Thread(target=livetraj, args=(q9,9,))
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()
    t8.start()
    t9.start()
    t10.start()
    t11.start()

    while True:
        # IP = "192.168.1.1"
        # print("start")
        # # IP = "10.0.0.1"
        # PORT_TO_MAX = 7980
        # sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # sock.bind((IP, PORT_TO_MAX))
        # data, server = sock.recvfrom(1024)
        # received = []
        # xP = [0,0]
        # x = [0, 0]
        # if data.startswith(b'int'):
        #     received = struct.unpack('!I', data[-4:])[0]
        # elif data.startswith(b'list'):
        #     for a in range(len(x)):
        #         if a == 0:
        #             x[a] = (repr(struct.unpack('!I', data[-4:])[0]))
        #         else:
        #             x[a] = (repr(struct.unpack('!I', data[-(4 * (a + 1)):-(4 * a)])[0]))
        # else:
        #     received = data.decode('utf-8')
        #
        # if x[0] != xP[0] or x[1] != xP[1]:
        #     x = xP
        x = q0.get()
        print(x)
        for s in range(len(coordinateScale)):
            d = findDistance(coordinateScale[s], x)
            if abs(lastdist[s] - d) > 10:
                distances[s] = d
                height = np.interp(d, (0, 127), (400, 0))
                if height > 400:
                    height = 400
                quay[s].put(height)
                lastdist[s] = d

        # dNP = np.array(distances)
        # heights = np.interp(dNP, (0, 127), (400, 0))
        # print(heights)
        # for a in range(len(heights)):
        #     heights









