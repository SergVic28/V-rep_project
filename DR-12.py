# -*- coding: utf-8 -*-
import vrep
from socket import *
import struct
import cv2 # OpenCV
import numpy as np
import array
from PIL import Image as I # Для работы с изображением
import time
import math


HOST = '127.0.0.1'

# Start simulation (CLIENT) <--- GUI
PORT_start = 20000
addr_in = (HOST, PORT_start)
socket_start = socket(AF_INET, SOCK_DGRAM)
socket_start.bind(addr_in)

# Camera (SERVER) ---> GUI
PORT_camera = 21000
addr_camera_GUI = (HOST, PORT_camera)
socket_camera_GUI = socket(AF_INET, SOCK_DGRAM)

# DR-12 ---> Odometry
PORT_odom = 23000
addr_robot_odom = (HOST, PORT_odom)
socket_robot_odom = socket(AF_INET, SOCK_DGRAM)

print ("Wait signal to connection")

leftMotorHandle = 0
vLeft = 0 #left velocity
rightMotorHandle = 0
vRight = 0 #right velocity
sensorHandle = [0,0,0,0,0,0,0,0]
noDetectionDist=0.75 # detection distance
maxDetectionDist=0.4 # distance on which the robot will go around the obstacle
detect=[0,0,0,0,0,0,0,0]
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6] #turn the wheels if the sensor is working
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2]
speed=2

start = 0

# Get start command
udp_data, address = socket_start.recvfrom(1024)
data = struct.unpack('b', udp_data)
start = data[0]
print (start)

# Функция
def follow_color_object(image): #color):
# Принимаем BGR изображение как массив Numpy
# Возвращаем координаты х,у центра, если объект найден
# (-1,-1) если центр не найден

    # Размытие изображения для уменьшения шумов
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Конвертируем BGR в HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([97,77,77])
    upper_blue = np.array([140,255,255])

    # Получаем синий цвет (маска)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Размытие маски
    blurmask = cv2.GaussianBlur(mask, (5,5),0)

    # Моменты изображения (сумма всех точек - пикселей)
    moments = cv2.moments((blurmask))
    m00 = moments['m00'] # Момент нулевого порядка - кол-во всех точек, сост. пятно
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00) # Момент первого порядка - сумма Х координат точек
        centroid_y = int(moments['m01']/m00) # Сумма Y координат

    # Предполагаем, что центра нет
    center = None #(-1,-1)

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:
        center = (centroid_x, centroid_y, m00)
    return center

while start == True:

    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19998, True, True, 5000, 5)
    print ("START")
    if clientID!=-1:
        print ("Connected to remote API server")
        # Init
        res, Vision_sensor = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        res, leftMotorHandle = vrep.simxGetObjectHandle(clientID, 'dr12_leftJoint_', vrep.simx_opmode_oneshot_wait)
        res, rightMotorHandle = vrep.simxGetObjectHandle(clientID, 'dr12_rightJoint_', vrep.simx_opmode_oneshot_wait)

        errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, Vision_sensor, 0, vrep.simx_opmode_streaming)
        time.sleep(1)

        if res != 0:
            print ("Can not find left or right motor")
        else:
            print ("Left and right motor connected")

        # Инициализируем ультрасоники
        for i in range(8):
            res, sensorHandle[i] = vrep.simxGetObjectHandle(clientID, "dr12_ultrasonicSensor%d" % (i + 1), vrep.simx_opmode_oneshot_wait)
            if res != 0:
                print ("Ultrasonic sensor dr12_ultrasonicSensor%d not found!" % (i + 1))
            else:
                print ("Connection to sensor dr12_ultrasonicSensor%d!" % (i + 1))
                res, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensorHandle[i], vrep.simx_opmode_streaming)

            print ("All sensors successfully detected")

        # Отклонение и скорость
        while vrep.simxGetConnectionId(clientID) != -1:
            # Info about angle
            res, rotL = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_oneshot_wait)
            res, rotR = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_oneshot_wait)

            # Send data to Odometry
            send_param = struct.pack('2d', rotL, rotR)
            socket_robot_odom.sendto(send_param, addr_robot_odom)

            for i in range(8):
                res, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensorHandle[i], vrep.simx_opmode_buffer)
                if res == 0:
                    dist = coord[2]
                    if state > 0 and dist < noDetectionDist:
                        if dist < maxDetectionDist:
                            dist = maxDetectionDist
                        detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
                    else:
                        detect[i] = 0
                else:
                    detect[i] = 0

            vLeft = speed
            vRight = speed

            for i in range(8):
                vLeft = vLeft + braitenbergL[i] * detect[i] * 0.5
                vRight = vRight + braitenbergR[i] * detect[i] * 0.5

            # Получаем изображение
            errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, Vision_sensor, 0, vrep.simx_opmode_buffer)
            if errorCode == vrep.simx_return_ok:
                image_byte_array = array.array('b', image)
                #print image_byte_array
                image_byte_array.reverse()
                image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
                image2 = np.asarray(image_buffer)
                cv2.flip(image2, 1, image2)
                socket_camera_GUI.sendto(image_byte_array, addr_camera_GUI)

                # Ищем объект заданного цвета
                Centroid = follow_color_object(image2)
                if Centroid == None:
                    res = vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming)
                    res = vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming)

                else:
                    print ("=== Object found ===")
                    # Добавляем маркер в виде прямоугольника, когда объект обнаружен
                    cv2.rectangle(image2, (Centroid[0]-15,Centroid[1]-15), (Centroid[0]+15,Centroid[1]+15), (0xff,0xf4,0x0d), 1)

                    center = follow_color_object(image2)
                    #print(center[2])
                    if center[0] != -1:
                        a = float( center[0] )
                        angle = -a/128*120+60
                    #print(-a/128*120+60, center[0])

                    if center[2] < 450000:
                        leftspeed = math.cos(angle*math.pi / 180)*speed + vLeft
                        rightspeed = speed + vRight
                        if angle <= 0:
                            rightspeed = math.cos(angle*math.pi / 180)*speed + vRight
                            leftspeed = speed + vLeft
                        res = vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, leftspeed, vrep.simx_opmode_streaming)
                        res = vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, rightspeed, vrep.simx_opmode_streaming)
                    else:
                        res = vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, vrep.simx_opmode_streaming)
                        res = vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, vrep.simx_opmode_streaming)
                        print ("Stop")

                cv2.imshow(' ', image2)
                cv2.waitKey(1)

            elif errorCode == vrep.simx_return_novalue_flag:
                print ("No image yet")
                pass
            else:
                print (errorCode)
    else:
        print ("Failed to connect to remote API Server")
        vrep.simxFinish(clientID)