import math
import time
from threading import Condition, Thread

import cv2
import requests
from networktables import NetworkTables
from numpy import array

cap = cv2.VideoCapture(0)
ROBOT_IP = '10.56.35.2'
smart_dashboard = None
CALIBRATION_PORT = 'tower'

min_hsv = array((60, 255, 77))
max_hsv = array((60, 255, 133))
max_area_diff = 0

camera_view_angle = 50

def process_image(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(frame, min_hsv, max_hsv)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total_x = 0
    total_y = 0
    cont = 0

    areas = [cv2.contourArea(c) for c in contours]

    if areas == []: return math.nan, math.nan

    max_area = max(areas)

    for i in range(len(contours)):
        c = contours[i]
        area = areas[i]

        if area < max_area - max_area_diff:
            continue

        M = cv2.moments(c)
        x = M['m10'] / M['m00']
        y = M['m01'] / M['m00']
        total_x += x
        total_y += y
        cont += 1

    if cont == 0:
        return math.nan, math.nan


    return frame.shape[1] / 2 - total_y / cont, (frame.shape[0] / 2 - total_x / cont) * camera_view_angle / frame.shape[0]

def put_number(key, number):
    if smart_dashboard:
        smart_dashboard.putNumber(key, number)

def init_smart_dashboard():
    if not smart_dashboard:
        return
    smart_dashboard.putNumber('vision_max_area_diff' + CALIBRATION_PORT, max_area_diff)
    smart_dashboard.putNumber('calibration_lower_h' + CALIBRATION_PORT, min_hsv[0])
    smart_dashboard.putNumber('calibration_lower_s' + CALIBRATION_PORT, min_hsv[1])
    smart_dashboard.putNumber('calibration_lower_v' + CALIBRATION_PORT, min_hsv[2])
    smart_dashboard.putNumber('calibration_upper_h' + CALIBRATION_PORT, max_hsv[0])
    smart_dashboard.putNumber('calibration_upper_s' + CALIBRATION_PORT, max_hsv[1])
    smart_dashboard.putNumber('calibration_upper_v' + CALIBRATION_PORT, max_hsv[2])
    smart_dashboard.putNumber('camera_view_angle' + CALIBRATION_PORT, camera_view_angle)

def update_vars():
    if not smart_dashboard:
        return

    global max_area_diff, camera_view_angle

    min_hsv[0] = smart_dashboard.getNumber('calibration_lower_h' + CALIBRATION_PORT, min_hsv[0])
    min_hsv[1] = smart_dashboard.getNumber('calibration_lower_s' + CALIBRATION_PORT, min_hsv[1])
    min_hsv[2] = smart_dashboard.getNumber('calibration_lower_v' + CALIBRATION_PORT, min_hsv[2])

    max_hsv[0] = smart_dashboard.getNumber('calibration_upper_h', max_hsv[0])
    max_hsv[1] = smart_dashboard.getNumber('calibration_upper_s' + CALIBRATION_PORT, max_hsv[1])
    max_hsv[2] = smart_dashboard.getNumber('calibration_upper_v' + CALIBRATION_PORT, max_hsv[2])
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
        return status_code == 200
    except:
        print('robot is dead 🦀')
        return False

if __name__ == '__main__':

    start_connection()
    while True:
        success, frame = cap.read()

        if not success:
            continue

        x, angle = process_image(frame)

        if x is not math.nan:
            put_number('vision_tower_x', x)
            put_number('vision_tower_angle', angle)
            print("x: ", x, "\nangle: ", angle)
        
