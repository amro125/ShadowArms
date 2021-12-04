#
# Simple test program for the Python Motion SDK.
#
# @file    sdk/python/test.py
# @version 2.5
#
# Copyright (c) 2017, Motion Workshop
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
from rtpmidi import RtpMidi
from pymidi import server
import sys
import MotionSDK
from xml.etree.ElementTree import XML
import time
from xarm.wrapper import XArmAPI
import os
import csv
import sys
from queue import Queue
from threading import Thread
import random
import numpy as np
import time

PortConsole = 32075
PortConfigurable = 32076
NSample = 10


class MyHandler(server.Handler):
    def on_peer_connected(self, peer):
        # Handler for peer connected
        print('Peer connected: {}'.format(peer))

    def on_peer_disconnected(self, peer):
        # Handler for peer disconnected
        print('Peer disconnected: {}'.format(peer))

    def on_midi_commands(self, peer, command_list):
        # Handler for midi msgs
        for command in command_list:
            chn = command.channel

            if chn == 13:  # this means its channel 14!!!!!
                if command.command == 'note_on':
                    # print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    print('key {} with velocity {}'.format(key, velocity))
                    params = [key, velocity]
                    danceq.put(params)
                    # step[key-1].start()


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        # a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, -1.0, 0.0, 1.309, 0.0, 0.88, 0.0], wait=False, speed=0.4, acceleration=0.25,
                          is_radian=True)


def parse_name_map(xml_node_list):
    name_map = {}

    tree = XML(xml_node_list)

    # <node key="N" id="Name"> ... </node>
    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map


def liveTraj(qtraj, position, robot):
    while True:
        receive = qtraj.get()
        position = receive[0]
        robot = receive[1]
        robot.set_mode(1)
        robot.set_state(0)
        q_i = robot.angles
        # arm2.angles
        # q_i = s[1]
        q_dot_i = 0
        q_dot_f = 0
        q_dotdot_i = 0
        q_dotdot_f = 0
        q_f = position
        i = 0
        tf = 3
        p = q_i[:]

        t_array = np.arange(0, tf, 0.006)
        print("start")

        while i <= len(t_array):
            for j in range(7):
                start_time = time.time()

                if i == len(t_array):
                    t = tf
                else:
                    t = t_array[i]
                a0 = q_i[j]
                a1 = q_dot_i
                a2 = 0.5 * q_dotdot_i
                a3 = 1.0 / (2.0 * tf ** 3.0) * (20.0 * (q_f[j] - q_i[j]) - (8.0 * q_dot_f + 12.0 * q_dot_i) * tf - (
                        3.0 * q_dotdot_f - q_dotdot_i) * tf ** 2.0)
                a4 = 1.0 / (2.0 * tf ** 4.0) * (30.0 * (q_i[j] - q_f[j]) + (14.0 * q_dot_f + 16.0 * q_dot_i) * tf + (
                        3.0 * q_dotdot_f - 2.0 * q_dotdot_i) * tf ** 2.0)
                a5 = 1.0 / (2.0 * tf ** 5.0) * (
                        12.0 * (q_f[j] - q_i[j]) - (6.0 * q_dot_f + 6.0 * q_dot_i) * tf - (
                            q_dotdot_f - q_dotdot_i) * tf ** 2.0)

                p[j] = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

            arm2.set_servo_angle_j(angles=p, is_radian=False)
            tts = time.time() - start_time
            sleep = 0.006 - tts

            if tts > 0.006:
                sleep = 0

            time.sleep(sleep)
            i += 1


def test_Client(host):
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
    left = [arm3]
    right = [arm2]
    rest1 = [arm2, arm5, arm7, arm9]

    rest = [arm1, arm4, arm6, arm8, arm10]
    left = [arm2, arm5, arm8]
    right = [arm3, arm7, arm10]
    head = [arm1, arm4, arm6]

    positions1 = [arm4, arm3, arm2, arm1]
    positions2 = [arm7, arm6, arm5]
    positions3 = [ arm10, arm9, arm8]
    positionq=[posq1, posq2,posq3,posq4]
    totalpositions = [positions1, positions2, positions3]
    client = MotionSDK.Client(host, PortConfigurable)
    row=0


    print("Connected to %s:%d" % (host, PortConfigurable))

    xml_string = \
        "<?xml version=\"1.0\"?>" \
        "<configurable inactive=\"1\">" \
        "<r/>" \
        "<c/>" \
        "</configurable>"

    if not client.writeData(xml_string):
        raise RuntimeError(
            "failed to send channel list request to Configurable service")

    num_frames = 0
    xml_node_list = None
    hips_key = None
    hips2_key = None
    shoulder_key = None
    hand_key = None
    Lhand_key = None


    followCount = 0
    lastpoint = 0.00
    counter = 0
    t_elapsed = 0
    arm = XArmAPI('192.168.1.203')

    arm.set_simulation_robot(on_off=False)
    arm.motion_enable(enable=True)
    arm.clean_warn()
    arm.clean_error()
    arm.set_mode(0)
    arm.set_state(0)

    arm.set_servo_angle(angle=[0.0, -1.0, 0.0, 1.309, 0.0, 0.88, 0.0], wait=True, speed=0.4, acceleration=0.25,
                        is_radian=True)
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    input("press enter to begin")
    time.sleep(5)
    map = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    q = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []
    q7 = []
    q8 = []
    q9 = []
    q10 = []
    q11 = []
    q12 = []
    q13 = []
    q14 = []
    q15 = []
    dance = []
    velocity = 1
    while True:
        # Block, waiting for the next sample.
        start = time.time()

        data = client.readData()
        if data is None:
            raise RuntimeError("data stream interrupted or timed out")
            break

        if data.startswith(b"<?xml"):
            xml_node_list = data
            continue

        container = MotionSDK.Format.Configurable(data)

        #
        # Consume the XML node name list.
        #
        if xml_node_list:
            name_map = parse_name_map(xml_node_list)

            flat_list = []
            for key in container:
                if key not in name_map:
                    raise RuntimeError(
                        "device missing from name map, unable to print "
                        "header")

                item = container[key]
                item2 = container[key]
                item3 = container[key]

                name = name_map[key]
                if "Head" == name:
                    hips_key = key
                if "Hips" == name:
                    hips2_key = key
                if "SpineLow" == name:
                    chestkey = key
                if "RightLeg" == name:
                    legkey = key
                if "LeftHand" == name:
                    Lhand_key = key
                if "RightHand" == name:
                    hand_key = key

                # print("{}:{}".format(key, name))

            # print(",".join(["{}".format(v) for v in flat_list]))

            if None == hips_key:
                raise RuntimeError("Hips sensor missing from name map")

            xml_node_list = None

        #
        # get the rotation and position values
        #
        handposition = []
        Lhandposition = []
        rotation = []

        position2 = []
        rotation2 = []

        rotation3 = []
        position3 = []
        legrotation = []
        handrotation = []
        Lhandrotation = []

        item = container[hips_key]
        item2 = container[hips2_key]
        item3 = container[chestkey]
        legitem = container[legkey]
        handitem = container[hand_key]
        Lhanditem = container[Lhand_key]

        # from https://www.motionshadow.com/download/media/configurable.xml
        # the channels requested come in order.  we requested r, c
        # r = [x,y,z]  Euler angle in radians
        # c = [w,x,y,z]  w is the constraint value (0 or 1, can ignore)
        # so, r = values [0:3], c = values [4,7] (ignore w value)

        # 1: Body
        # 2: Chest
        # 3: Head
        # 4: HeadEnd
        # 5: Hips
        # 6: LeftArm
        # 7: LeftFinger
        # 8: LeftFingerEnd
        # 9: LeftFoot
        # 10: LeftForearm
        # 11: LeftHand
        # 12: LeftHeel
        # 13: LeftLeg
        # 14: LeftShoulder
        # 15: LeftThigh
        # 16: LeftToe
        # 17: LeftToeEnd
        # 18: Neck
        # 19: RightArm
        # 20: RightFinger
        # 21: RightFingerEnd
        # 22: RightFoot
        # 23: RightForearm
        # 24: RightHand
        # 25: RightHeel
        # 26: RightLeg
        # 27: RightShoulder
        # 28: RightThigh
        # 29: RightToe
        # 30: RightToeEnd
        # 31: SpineLow
        for i in range(0, 3):
            rotation.append((item.value(i)))
            rotation2.append((item2.value(i)))
            rotation3.append((item3.value(i)))
            legrotation.append((legitem.value(i)))
            handrotation.append((handitem.value(i)))
            Lhandrotation.append((Lhanditem.value(i)))
        for i in range(4, 7):
            handposition.append(handitem.value(i))
            Lhandposition.append(Lhanditem.value(i))
        #     position2.append(item2.value(i))
        #     position3.append(item3.value(i))
        if velocity == 1:
            offset = rotation[0]
            offset2 = rotation[1]
            offset3 = rotation2[1]
            offset4 = rotation3[0]
            offset5 = rotation2[0]
            legoffset = legrotation[0]
            leanoffset = rotation3[2]
            handoffset1 = handrotation[0]
            handoffset2 = handrotation[2]
            Lhandoffset1 = Lhandrotation[0]
            Lhandoffset2 = Lhandrotation[2]
            handoffsetx = handposition[0]
            handoffsety = handposition[1]
            handoffsetz = handposition[2]
            Lhandoffsetx = Lhandposition[0]
            Lhandoffsety = Lhandposition[1]
            Lhandoffsetz = Lhandposition[2]

        mapangle = rotation[0] - offset
        mapangle2 = rotation[1] - offset2
        mapangle3 = rotation2[1] - offset3
        mapangle5 = rotation3[0] - offset4
        mapleg = legrotation[0] - legoffset
        handtwist = handrotation[0] - handoffset1
        handup = handrotation[2] - handoffset2
        Lhandtwist = Lhandrotation[0] - Lhandoffset1
        Lhandup = Lhandrotation[2] - Lhandoffset2
        handx = handposition[0] - handoffsetx
        handy = handposition[1] - handoffsety
        handz = handposition[2] - handoffsetz
        Lhandx = Lhandposition[0] - Lhandoffsetx
        Lhandy = Lhandposition[1] - Lhandoffsety
        Lhandz = Lhandposition[2] - Lhandoffsetz

        # maplean = rotation3[2] -leanoffset
        # mapangle5 = (rotation2[0] - offset5)
        q.append(mapangle)
        q2.append(mapangle2)
        q3.append(mapangle3)
        q5.append(mapangle5)
        q4.append(mapleg)
        q6.append(handtwist)
        q7.append(handup)
        q8.append(Lhandtwist)
        q9.append(Lhandup)
        q10.append(handx)
        q11.append(handy)
        q12.append(handz)
        q13.append(Lhandx)
        q14.append(Lhandy)
        q15.append(Lhandz)
        windows = [q, q2, q3, q5, q4, q6, q7, q8, q9, q10, q11, q12, q13, q14, q15]
        if counter == 2500:
            print("go")
        if counter >= 2500:
            for joint in range(len(windows)):
                windows[joint] = windows[joint][-20:]
                map[joint] = np.mean(windows[joint])
        if danceq.empty() == False:
            param = danceq.get()
            velocity = param[1]
        if velocity == 2:
            arm.set_servo_angle_j(
                angles=[0.0 - map[2]/2, -1.0 - (map[4] / 5), 0.0, 1.309 - (map[3] * 5.5), -map[1], 0.88 + 2 * map[0],
                        0.0], is_radian=True)

        if velocity == 20:
            print("swap")

        if velocity == 3:
            dance.append(map[:])
            arm.set_servo_angle_j(
                angles=[0.0 - map[2]/2, -1.0 - (map[4] / 5), 0.0, 1.309 - (map[3] * 5.5), -map[1], 0.88 + 2 * map[0],
                        0.0], is_radian=True)
            Dance_offset = 0

        if velocity == 4:
            if Dance_offset == 0:
                Calibrate2 = map[:]
                print("readiong")



            else:
                otherdo = np.subtract(map, Calibrate2)
                # print(Calibrate2)
                arm.set_servo_angle_j(
                    angles=[0.0 - Calibrate2[2]/2, -1.0 - (map[4] / 5), 0.0, 1.309 - (map[3] * 5.5), -map[1],
                            0.88 + 2 * map[0],
                            0.0], is_radian=True)
                for a in rest1:
                    a.set_servo_angle_j(angles=[0.0, -1.0, 0.0, 1.309, otherdo[1], 0.88 + otherdo[0] * 2, 0.0],
                                        is_radian=True)
                for a in rest:
                    # dancetodo = np.subtract(dance[Dance_offset], dance[0])
                    a.set_servo_angle_j(angles=[0.0, -1.0, 0.0, 1.309, -otherdo[1], 0.88 + otherdo[0] * 2, 0.0],
                                        is_radian=True)
                #     print("sending")
            dance.append(map[:])
            Dance_offset += 1

        if velocity == 5:
            calibrate = map[:]
            for a in arms:
                a.set_simulation_robot(on_off=False)
                # a.motion_enable(enable=True)
                a.clean_warn()
                a.clean_error()
                a.set_mode(0)
                a.set_state(0)

                a.set_servo_angle(angle=[0.0, -45, 0.0, 45, 0.0, 0.0, 0.0], wait=False, speed=32, acceleration=2,
                                  is_radian=False)
            param = danceq.get()
            velocity = param[1]

            calibratepos = 0

        if velocity == 6:
            if calibratepos == 0:
                for a in arms:
                    a.set_mode(1)
                    a.set_state(0)

                calibratepos = 1
                calibrated = map[:]

            else:
                handpos = np.subtract(map, calibrated)

                for a in left:
                    # print(handpos[7],handpos[8])
                    a.set_servo_angle_j(angles=[0.0, -45 + handpos[12] / 10, 0.0 + handpos[13] / 10,
                                                45 + handpos[12] / 10 + handpos[14] / 10, -handpos[8] * 40,
                                                handpos[7] * 40, 0.0],
                                        is_radian=False)
                for a in right:
                    a.set_servo_angle_j(angles=[0.0, -45 + handpos[12] / 10, 0.0 - handpos[13] / 10,
                                                45 + handpos[12] / 10 + handpos[14] / 10, 0.0,
                                                0.0, 0.0],
                                        is_radian=False)
                for a in head:
                    a.set_servo_angle_j(angles=[0.0, -45 , 0.0, 45, -handpos[8] * 40,
                                                handpos[7] * 40, 0.0], is_radian=False)
            followCount = 0


        if velocity == 7:


        if velocity == 8:
            spots=[[-55,-70,0,90,0,0,0],[-75,-95,-70,60,0,0,0],[0,80,0,65,-10,165,0],[-92,80,-140,65,-80,120,0],[-180,25,-170,140,-10,30,0],[-141,100,300,60,-150,-21,0],[-151,-50,-75,70,-180,-21,0],[-60,-100,-80,180,-180,50,0]]
            while velocity == 8:
                param = danceq.get()
                velocity = param[1]
                check = [map[2],map[4], 0, map[3], map[1], map[0], 0]
                close = np.zeros(len(totalpositions[row]))
                for x in range(len(spots)):
                    diff = np.subtract(spots, check)
                    close[x]= np.linalg.norm(diff)
                bestpos = np.amin(close)
                # spots[bestpos]
                for x in range(len(totalpositions[row])):
                    goto = ((x+1)/4)*np.array(spots[bestpos])
                    positionq[x].put([goto, totalpositions[row][x]])
                    time.sleep(0.25)
                row += 1





        #
        counter += 1

        # print("r: " + ",".join(["{}".format(round(v, 8)) for v in rotation]))
        # print("c: " + ",".join(["{}".format(round(v, 8)) for v in position]))

        num_frames += 1


def test_LuaConsole(host):
    client = MotionSDK.Client(host, PortConsole)

    print("Connected to %s:%d" % (host, port))

    #
    # General Lua scripting interface.
    #
    lua_chunk = \
        "if not node.is_reading() then" \
        "   node.close()" \
        "   node.scan()" \
        "   node.start()" \
        " end" \
        " if node.is_reading() then" \
        "   print('Reading from ' .. node.num_reading() .. ' device(s)')" \
        " else" \
        "   print('Failed to start reading')" \
        " end"

    # print(LuaConsole.SendChunk(client, lua_chunk, 5))

    # Scripting language compatibility class. Translate Python calls into Lua
    # calls and send them to the console service.
    node = LuaConsole.Node(client)
    # print("node.is_reading() = {}".format(node.is_reading()))


def main(argv):
    # Set the default host name parameter. The SDK is socket based so any
    # networked Motion Service is available.
    host = ""
    if len(argv) > 1:
        host = argv[1]
    global danceq
    global posq1
    global posq2
    global posq3
    global posq4
    danceq = Queue()
    posq1 = Queue()
    posq2 = Queue()
    posq3 = Queue()
    posq4 = Queue()
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

    global arms
    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9, arm10]

    totalArms = len(arms)

    setup()

    def test():
        ROBOT = "xArms"
        PORT = 5004
        rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
        rtp_midi.run()

    ROBOT = "xArms"
    PORT = 5004
    rtpthread = Thread(target=test, args=())
    rtpthread.start()

    livetraj1 = Thread(target = liveTraj, args=())
    livetraj2 = Thread(target=liveTraj, args=())
    livetraj3 = Thread(target=liveTraj, args=())
    livetraj4 = Thread(target=liveTraj, args=())
    livetraj1.start()
    livetraj2.start()
    livetraj3.start()
    livetraj4.start()

    # test_LuaConsole(host)
    test_Client(host)

    # Requires a data file. Do not test by default.
    # test_File()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
