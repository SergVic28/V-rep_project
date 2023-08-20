# -*- coding: utf-8 -*-
from socket import *
import struct
import numpy as np
import math


HOST = '127.0.0.1'

# Odometry <--- DR-12
PORTin = 23000
addr_robot_odom = (HOST, PORTin)
socket_robot_odom = socket(AF_INET, SOCK_DGRAM)
socket_robot_odom.bind(addr_robot_odom)

# Odometry ---> GUI
PORTout = 22000
addr_odom_GUI = (HOST, PORTout)
socket_odom_GUI = socket(AF_INET, SOCK_DGRAM)

angleWL = 0.0 # угол поворота левого колеса
angleWR = 0.0 # угол поворота правого колеса
Rot = 0.0 # угол курса
SR = 0.0 # дистанция правого колеса
SL = 0.0 # дистанция левого колеса
dSR = 0.0 # расстояние, которое проходит колесо
dSL = 0.0
dSD = 0.0
position = np.array([[0.0, 0.0], [0.0, 0.0]])
R = 0.043 # радиус колеса
L = 0.154 # расстояние между колесами

# Рассчитываем угол поворота робота
def get_angle(angle, self_angle):
    xa = [np.cos(angle), np.sin(angle)] #вектор
    rot = np.array([[np.cos(-self_angle), -np.sin(-self_angle)], #матрица поворота вокруг z
                    [np.sin(-self_angle), np.cos(-self_angle)]])
    delta_v = np.dot(rot, xa) # угол поворота робота
    delta = np.arctan2(delta_v[1], delta_v[0])
    return delta

# Рассчитываем позицию робота
def Position(robot_angleWL, robot_angleWR):
    global Rot, angleWL, angleWR
    position[1] = position[0] # xy_start
    dAngleWL = get_angle(robot_angleWL, angleWL) #приращение угла
    dAngleWR = get_angle(robot_angleWR, angleWR)
    angleWL += dAngleWL
    angleWR += dAngleWR
    dSL = dAngleWL * R #путь левого колеса
    dSR = dAngleWR * R
    dSD = (dSL + dSR) / 2 #пройденный путь
    SL = angleWL * R # вращение
    SR = angleWR * R
    Rot = (SR - SL) / L #полный угол поворота колес
    position[0] = [np.cos(Rot) * dSD, np.sin(Rot) * dSD] + position[1] # xy_end
    return position[0, 0], position[0, 1], Rot #x0 x1 угол

print ("Wait data")

while True:
    Robot_data, addr_robot_odom = socket_robot_odom.recvfrom(1024)
    rotL, rotR = struct.unpack('2d', Robot_data)
    if rotL < 0: #Recalculation to the angular velocity
        rotL = 2 * math.pi + rotL
    if rotR < 0:
        rotR = 2 * math.pi + rotR
    robot_x, robot_y, robot_angle = Position(rotL, rotR) # Calculation position of robot
    robot_angle = (robot_angle * 180) / math.pi
    robot_param = struct.pack('3d', robot_x, robot_y, robot_angle) # Send data to GUI
    socket_odom_GUI.sendto(robot_param, addr_odom_GUI)
