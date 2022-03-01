import math
import time
import logging
from logging import exception
from threading import Condition, Thread

import cv2
import numpy as np
import requests
from cscore import CameraServer
from networktables import NetworkTables
from numpy import array

# Camera Settings
cs = CameraServer()
camera = cs.startAutomaticCapture()
input_stream = cs.getVideo()
output_stream = cs.putVideo('Vision', 320, 240)

# Templates
clean_img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

# Connection Parameters
# cap = cv2.VideoCapture(0)
ROBOT_IP = 'http://10.56.35.2'
smart_dashboard = None
CALIBRATION_PORT = 'tower'

# Color Range
min_hsv = array((60, 255, 77))
max_hsv = array((78, 255, 133))

# Hardware Parameters
camera_view_angle = 50

# Logger
logging.basicConfig(filename='gamelog.log', level=logging.info, filemode='w', format='%(levelname)s: %(message)s')


def process_image(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # TODO: Find solution for camera placement
    frame[0:frame.shape[0], frame.shape[1]-40:frame.shape[1]] = [0, 0, 0]

    mask = cv2.inRange(frame, min_hsv, max_hsv)

    # mask = cv2.erode(mask, kernel, iterations=1)
    # mask = cv2.dilate(mask, kernel, iterations=1)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total_x = 0
    total_y = 0
    cont = 0
    # print('min_hsv:', min_hsv)
    # print('max_hsv:', max_hsv)
    areas = [(cv2.contourArea(c) for c in contours) if contours else []]
    # print('area amount:', len(areas))
    if not bool(areas):
        return math.nan, math.nan

    cords = np.array()

    for i in range(len(contours)):
        c = contours[i]

        M = cv2.moments(c)
        if M['m00'] == 0.0:
            continue
        x = M['m10'] / M['m00']
        y = M['m01'] / M['m00']
        cords = np.append(cords, [x, y])
        total_x += x
        total_y += y
        cont += 1
            
    if cont > 0:
        cords = cords[cords[:, 0].argsort()]
        x_list = cords[:, 0]
        xn = [np.count_nonzero((x_list >= i) & (x_list < i + 40)) for i in x_list]
        i = np.argmax(xn)
        x = np.min(cords[i:i+xn[i], 0])
        y = np.average(cords[i:i+xn[i], 1])
        cv2.circle(mask, (int(x), int(y)), 10, 255, 2)
    
    output_stream.putFrame(mask)
    # print('count:', cont)
    if cont == 0:
        return math.nan, math.nan
    return total_y / cont, total_x / cont


def put_number(key, number):
    if smart_dashboard:
        smart_dashboard.putNumber(key, number)


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
    logging.info(info, '; Connected=%s' % connected)
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

    start_connection()

    while True:
        ret, frame = input_stream.grabFrame(clean_img)
        # success, frame = cap.read()

        if ret == 0:
            continue
        # if not success:
        #     continue

        y, x = process_image(frame)

        if y is not math.nan:
            put_number('vision_tower_x', x)
            put_number('vision_tower_y', y)
        else:
            put_number('vision_tower_x', 1000)
            put_number('vision_tower_y', 1000)
