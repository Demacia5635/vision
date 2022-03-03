import logging
import math
import sys
import time
from logging import exception
from threading import Condition, Thread

import cv2
import numpy as np
import requests
from cscore import CameraServer
from networktables import NetworkTables
from numpy import array

# Camera Settings
width, height = 960, 540
cs = CameraServer()
camera = cs.startAutomaticCapture()
camera.setFPS(10)
camera.setResolution(width, height)
input_stream = cs.getVideo()
output_stream = cs.putVideo('Vision', width, height)


# Templates
clean_img = np.zeros(shape=(width, height, 3), dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)


# Connection Parameters
ROBOT_IP = 'http://10.56.35.2'
smart_dashboard = None
CALIBRATION_PORT = 'tower'


# Color Range
min_hsv = array((60, 255, 77))
max_hsv = array((78, 255, 133))


# Hardare Parameters
camera_view_angle = 50


def process_image(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #frame[0:frame.shape[0], frame.shape[1]-40:frame.shape[1]] = [0, 0, 0]

    mask = cv2.inRange(frame, min_hsv, max_hsv)

    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    logging.debug('min_hsv:', min_hsv)
    logging.debug('max_hsv:', max_hsv)

    areas = [cv2.contourArea(c) for c in contours]
    logging.debug('area amount:', len(areas))

    if not areas:
        return None, None

    length = len(areas)
    cords = np.empty((length, 2), dtype=np.float32)

    for i in range(length):
        c = contours[i]

        M = cv2.moments(c)
        if M['m00'] == 0.0:
            continue
        x = M['m10'] / M['m00']
        y = M['m01'] / M['m00']
        cords[i] = (x, y)
            
    cords = cords[cords[:, 0].argsort()]
    x_list = cords[:, 0]
    xn = [np.count_nonzero((x_list >= i) & (x_list < i + 40)) for i in x_list]
    i = np.argmax(xn)
    x = np.min(cords[i:i+xn[i], 0])
    y = np.average(cords[i:i+xn[i], 1])
    cv2.circle(mask, (int(x), int(y)), 10, 255, 2)
    output_stream.putFrame(mask)

    return x, y


def put_number(key, number):
    if smart_dashboard:
        smart_dashboard.putNumber(key, number)

def put_boolean(key, value):
    if smart_dashboard:
        smart_dashboard.putBoolean(key, value)


def init_smart_dashboard():
    if not smart_dashboard:
        return
    smart_dashboard.putNumber('calibration-lower-h-' + CALIBRATION_PORT, min_hsv[0])
    smart_dashboard.putNumber('calibration-lower-s-' + CALIBRATION_PORT, min_hsv[1])
    smart_dashboard.putNumber('calibration-lower-v-' + CALIBRATION_PORT, min_hsv[2])
    smart_dashboard.putNumber('calibration-upper-h-' + CALIBRATION_PORT, max_hsv[0])
    smart_dashboard.putNumber('calibration-upper-s-' + CALIBRATION_PORT, max_hsv[1])
    smart_dashboard.putNumber('calibration-upper-v-' + CALIBRATION_PORT, max_hsv[2])
    smart_dashboard.putNumber('camera_view_angle' + CALIBRATION_PORT, camera_view_angle)


def update_vars():
    if not smart_dashboard: return

    global camera_view_angle

    min_hsv[0] = smart_dashboard.getNumber('calibration-lower-h-' + CALIBRATION_PORT, min_hsv[0])
    min_hsv[1] = smart_dashboard.getNumber('calibration-lower-s-' + CALIBRATION_PORT, min_hsv[1])
    min_hsv[2] = smart_dashboard.getNumber('calibration-lower-v-' + CALIBRATION_PORT, min_hsv[2])

    max_hsv[0] = smart_dashboard.getNumber('calibration-upper-h-' + CALIBRATION_PORT, max_hsv[0])
    max_hsv[1] = smart_dashboard.getNumber('calibration-upper-s-' + CALIBRATION_PORT, max_hsv[1])
    max_hsv[2] = smart_dashboard.getNumber('calibration-upper-v-' + CALIBRATION_PORT, max_hsv[2])
    camera_view_angle = smart_dashboard.getNumber('camera_view_angle' + CALIBRATION_PORT, camera_view_angle)


def connect():
    global smart_dashboard
    cond = Condition()
    notified = False
    NetworkTables.initialize(server='10.56.35.2')
    NetworkTables.addConnectionListener(lambda connected, info: connection_listener(connected, info, cond), immediateNotify=True)

    with cond:
        logging.info("Connecting...")
        if not notified:
            cond.wait()

    logging.info("Connected!")
    smart_dashboard = NetworkTables.getTable('SmartDashboard')
    update_vars()
    init_smart_dashboard()
    smart_dashboard.addEntryListener(lambda source, key, value, isNew : update_vars())


def connection_listener(connected, info, cond : Condition):
    logging.info("{info}; Connceted={status}".format(info = info, status = connected))
    with cond:
        cond.notify()


def connect_periodically(on_connect = None):
    while not connected_to_robot():
        time.sleep(10)
    connect()
    if on_connect:
        on_connect()


def start_connection(on_connect = None):
    thread = Thread(target=lambda : connect_periodically(on_connect))
    thread.start()

def connected_to_robot():
    try:
        logging.info("Searching for robot...")
        status_code = requests.get(ROBOT_IP, timeout=(2, 1)).status_code
        logging.info(status_code)
        return status_code == 200
    except exception as e:
        logging.warning(e)
        logging.warning('robot is dead 🦀')
        return False


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.INFO)
    
    start_connection()

    while True:
        time, frame = input_stream.grabFrame(clean_img)

        if time == 0:
            continue

        x, y = process_image(frame)

        if x and y is not None:
            put_number('vision_tower_x', x)
            put_number('vision_tower_y', y)
            put_boolean('vision_found', True)
        else:
            put_boolean('vision_found', False)
