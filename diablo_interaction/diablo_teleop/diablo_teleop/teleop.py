#!/usr/bin/env python3
from http.client import OK
import rclpy
import time
import sys
import tty
import termios
import threading
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl


CMD_GO_FORWARD = 0x08
CMD_GO_LEFT = 0x04
CMD_ROLL_RIGHT = 0x09

CMD_HEIGH_MODE = 0x01
CMD_BODY_UP = 0x11

CMD_STAND_UP = 0x02
CMD_STAND_DOWN  =  0x12

CMD_PITCH = 0x03
CMD_PITCH_MODE = 0x13

CMD_SPEED_MODE = 0x05

print("Teleop start now!")
print("Press '`' to exit!")

keyQueue = []
old_setting = termios.tcgetattr(sys.stdin)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)


t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()

def main(args=None):
    rclpy.init(args=args) 
    node = Node("diablo_teleop_node")  
    msg = MotionCtrl()
    teleop_cmd = node.create_publisher(MotionCtrl,"diablo/MotionCmd",2)

    while True:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
            if key == 'w':
                msg.cmd_id = CMD_GO_FORWARD
                msg.value = 1.0
            elif key == 's':
                msg.cmd_id = CMD_GO_FORWARD
                msg.value = -(1.0)
            elif key == 'a':
                msg.cmd_id = CMD_GO_LEFT
                msg.value = 1.0
            elif key == 'd':
                msg.cmd_id = CMD_GO_LEFT
                msg.value = -(1.0)
            elif key == 'e':
                msg.cmd_id = CMD_ROLL_RIGHT
                msg.value = 0.1
            elif key == 'q':
                msg.cmd_id = CMD_ROLL_RIGHT
                msg.value = -(0.1)
            elif key == 'r':
                msg.cmd_id = CMD_ROLL_RIGHT
                msg.value = 0.0

            elif key == 'h':
                msg.cmd_id = CMD_BODY_UP
                msg.value = -(0.5)
            elif key == 'j':
                msg.cmd_id = CMD_BODY_UP
                msg.value = 1.0
            elif key == 'k':
                msg.cmd_id = CMD_BODY_UP
                msg.value = 0.5
            elif key == 'l':
                msg.cmd_id = CMD_BODY_UP
                msg.value = 0.0
                
            elif key == 'u':
                msg.cmd_id = CMD_PITCH
                msg.value = 0.5
            elif key == 'i':
                msg.cmd_id = CMD_PITCH
                msg.value = 0.0
            elif key == 'o':
                msg.cmd_id = CMD_PITCH
                msg.value = -(0.5)

            elif key == 'f':
                msg.cmd_id = CMD_SPEED_MODE
                msg.value = 1.0
            elif key == 'g':
                msg.cmd_id = CMD_SPEED_MODE
                msg.value = 0.0
            elif key == 'v':
                msg.cmd_id = CMD_HEIGH_MODE
                msg.value = 1.0
            elif key == 'b':
                msg.cmd_id = CMD_HEIGH_MODE
                msg.value = 0.0
            elif key == 'n':
                msg.cmd_id = CMD_PITCH_MODE
                msg.value = 1.0
            elif key == 'm':
                msg.cmd_id = CMD_PITCH_MODE
                msg.value = 0.0

            elif key == 'z':
                msg.cmd_id = CMD_STAND_UP
                msg.value = 0.0
            elif key == 'x':
                msg.cmd_id = CMD_STAND_DOWN
                msg.value = 0.0

            elif key == '`':
                break
        else:
            msg.cmd_id = 0
            msg.value = 0.0

        teleop_cmd.publish(msg)
        time.sleep(0.04)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    print('exit!')
    rclpy.shutdown() 


