import math
import time
from logging import exception
from threading import Condition, Thread

import cv2
import numpy as np
import requests
from cscore import CameraServer
from networktables import NetworkTables
from numpy import array

cs = CameraServer()
camera = cs.startAutomaticCapture()
input_stream = cs.getVideo()
output_stream = cs.putVideo('Vision', 320, 240)

clean_img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

# cap = cv2.VideoCapture(0)
ROBOT_IP = 'http://10.56.35.2'
smart_dashboard = None
CALIBRATION_PORT = 'tower'

min_hsv = array((60, 255, 77))
max_hsv = array((78, 255, 133))
max_area_diff = 1234567890

camera_view_angle = 50


def process_image(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
    areas = [cv2.contourArea(c) for c in contours]
    # print('area amount:', len(areas))
    if areas == []:
        return math.nan, math.nan

    max_area = max(areas)
    cords = {}

    for i in range(len(contours)):
        c = contours[i]
        area = areas[i]

        if area < 0.3 * max_area:
            continue

        M = cv2.moments(c)
        if M['m00'] == 0.0:
            continue
        x = M['m10'] / M['m00']
        y = M['m01'] / M['m00']
        cords[int(x)] = int(y)
        total_x += x
        total_y += y
        cont += 1
            
    if cont > 0:
        cords = sorted(cords.items())
        x_cords = [x_tuple[0] for x_tuple in cords]
        y_cords = [y_tuple[1] for y_tuple in cords]
        best_x = x_cords[0]
        best_y = y_cords[0]
        if len(x_cords) >= 2:
            if x_cords[1] - x_cords[0] <= 10:
                best_x = int((x_cords[1] - x_cords[0]) / 2)
                best_y = int((y_cords[1] - y_cords[0]) / 2)
        cv2.circle(mask, (best_x, best_y), 10, 255, 2)
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
    smart_dashboard.putNumber('vision_max_area_diff' + CALIBRATION_PORT, max_area_diff)
    smart_dashboard.putNumber('calibration-lower-h-' + CALIBRATION_PORT, min_hsv[0])
    smart_dashboard.putNumber('calibration-lower-s-' + CALIBRATION_PORT, min_hsv[1])
    smart_dashboard.putNumber('calibration-lower-v-' + CALIBRATION_PORT, min_hsv[2])
    smart_dashboard.putNumber('calibration-upper-h-' + CALIBRATION_PORT, max_hsv[0])
    smart_dashboard.putNumber('calibration-upper-s-' + CALIBRATION_PORT, max_hsv[1])
    smart_dashboard.putNumber('calibration-upper-v-' + CALIBRATION_PORT, max_hsv[2])
    smart_dashboard.putNumber('camera_view_angle' + CALIBRATION_PORT, camera_view_angle)

def update_vars():
    if not smart_dashboard:
        return

    global max_area_diff, camera_view_angle

    min_hsv[0] = smart_dashboard.getNumber('calibration-lower-h-' + CALIBRATION_PORT, min_hsv[0])
    min_hsv[1] = smart_dashboard.getNumber('calibration-lower-s-' + CALIBRATION_PORT, min_hsv[1])
    min_hsv[2] = smart_dashboard.getNumber('calibration-lower-v-' + CALIBRATION_PORT, min_hsv[2])

    max_hsv[0] = smart_dashboard.getNumber('calibration-upper-h-' + CALIBRATION_PORT, max_hsv[0])
    max_hsv[1] = smart_dashboard.getNumber('calibration-upper-s-' + CALIBRATION_PORT, max_hsv[1])
    max_hsv[2] = smart_dashboard.getNumber('calibration-upper-v-' + CALIBRATION_PORT, max_hsv[2])
    camera_view_angle = smart_dashboard.getNumber('camera_view_angle' + CALIBRATION_PORT, camera_view_angle)

    max_area_diff = smart_dashboard.getNumber('vision_max_area_diff' + CALIBRATION_PORT, max_area_diff)

def connect():
    global smart_dashboard
    cond = Condition()
    notified = False
    NetworkTables.initialize(server='10.56.35.2')
    NetworkTables.addConnectionListener(lambda connected, info: connection_listener(connected, info, cond), immediateNotify=True)

    with cond:
        print("Connecting...")
        if not notified:
            cond.wait()

    print("Connected!")
    smart_dashboard = NetworkTables.getTable('SmartDashboard')
    update_vars()
    init_smart_dashboard()
    smart_dashboard.addEntryListener(lambda source, key, value, isNew : update_vars())


def connection_listener(connected, info, cond : Condition):
    print(info, '; Connected=%s' % connected)
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
        print("Searching for robot...")
        status_code = requests.get(ROBOT_IP, timeout=(2, 1)).status_code
        print(status_code)
        return status_code == 200
    except exception as e:
        print(e)
        print('robot is dead 🦀')
        return False


if __name__ == '__main__':

    start_connection()

    while True:
        time, frame = input_stream.grabFrame(clean_img)
        # success, frame = cap.read()

        if time == 0:
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
