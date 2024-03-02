#!/usr/bin/env python
# coding=utf-8

# F1 - Auto/Manual
# F2 - Side/Behind
# F3 - Balanced/Center
# F4 - Distance ON/OFF
# F5 - Record Start/Stop
# Q  - Quit

import os
import signal
import time
import psutil
import array as arr
from pynput import keyboard

status = arr.array('i', [1, 1, 1, 1, 1])
record_dir = "/home/sj/record/"
record_start_time = 0

def get_pid_by_name(name):
    pid = None
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.info['name'] == name:
            pid = proc.info['pid']
            break
    return pid

def signal_handler(signal, frame):
    print('Received SIGINT signal')

def send_data():
    cmd = "cansend can0 19eeeeef#{:0>2d}{:0>2d}{:0>2d}{:0>2d}".format(status[0], status[1],status[2],status[3])
    print(cmd)
    os.system(cmd)

def on_press(key):
    print('\n\n====== {0} pressed ======'.format(key))

def on_release(key):
    try:
        if key.char == 'q':
            print('Exit')
            return False
    except AttributeError:    
        if key == keyboard.Key.f1:
            if status[0] == 1:
                print('\nAuto mode')
                status[0] = 2
            else:
                print('\nManual mode')
                status[0] = 1
            send_data()
        elif key == keyboard.Key.f2:
            if status[1] == 1:
                print('\nSide mode')
                status[1] = 2
            else:
                print('\nBehind mode')
                status[1] = 1
            send_data()
        elif key == keyboard.Key.f3:
            if status[2] == 1:
                print('\nBalanced mode')
                status[2] = 2
            else:
                print('\nCenter mode')
                status[2] = 1
            send_data()
        elif key == keyboard.Key.f4:
            if status[3] == 1:
                print('\nDistance ON')
                status[3] = 2
            else:
                print('\nDistance OFF')
                status[3] = 1
            send_data()
        elif key == keyboard.Key.f5:
            global record_start_time
            if (time.time() - record_start_time) > 3:
                record_start_time = time.time()
                if status[4] == 1:
                    print('\nStart record...')
                    status[4] = 2
                    
                    os.popen('rosbag record -o {0} /vsemi_tof_harv/camera/cloud /vsemi_tof_harv/camera/video /vsemi_tof_harv/camera/camera /vsemi_tof_harv/camera/sprout'.format(record_dir))
                    # os.popen('sleep 10 &')
                else:
                    print('\nStop record...')
                    status[4] = 1
                    pid = get_pid_by_name('rosbag')
                    
                    print('\npid: {0}'.format(pid))
                    if pid != None:
                        os.kill(pid, signal.SIGINT)
                    else:
                        print('Not found rosbag!')
            else:
                print('\033[31m \n\n\n\nERROR: Frequent operations\n\n \033[0m ')
        elif key == keyboard.Key.up:
            print('\nUP')
            os.system("cansend can0 18eeeeef#000100000000")
        elif key == keyboard.Key.down:
            print('\nDOWN')
            os.system("cansend can0 18eeeeef#000200000000")
        elif key == keyboard.Key.left:
            print('\nLEFT')
            os.system("cansend can0 18eeeeef#400000000000")
        elif key == keyboard.Key.right:
            print('\nRIGHT')
            os.system("cansend can0 18eeeeef#800100000000")
    
    print('====== {0} release ======'.format(key))

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release)as listener:
    listener.join()

# ...or,in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

