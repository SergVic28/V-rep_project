# -*- coding: utf-8 -*-
import vrep
from Tkinter import *
from socket import *
import struct
import array
from PIL import ImageTk
import PIL.Image as img
from threading import Thread
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

HOST = '127.0.0.1'

# Start simulation (SERVER) ---> DR-12
PORT_start = 20000
addr_out = (HOST, PORT_start)
socket_start = socket(AF_INET, SOCK_DGRAM)

# Camera (CLIENT) <--- DR-12
PORT_camera = 21000
addr_camera_GUI = (HOST, PORT_camera)
socket_camera_GUI = socket(AF_INET, SOCK_DGRAM)
socket_camera_GUI.bind(addr_camera_GUI)

# Recife from Odometry
PORT_odom = 22000
addr_odom_GUI = (HOST, PORT_odom)
socket_odom_GUI = socket(AF_INET, SOCK_DGRAM)
socket_odom_GUI.bind(addr_odom_GUI)


class GUI(Thread):
    def __init__(self, time, canvas_vrep):
        Thread.__init__(self)
        self.isrun = False
        self.exit = True
        self.canvas_vrep = canvas_vrep
        self.resolution = [128, 128]
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
        if (self.clientID != -1):
            print ("Connected to remote API server")

    def run(self):
        while (self.exit):
            if (self.isrun):
                while (vrep.simxGetConnectionId(self.clientID) != -1):
                    image_byte_array, addr_camera_GUI = socket_camera_GUI.recvfrom(
                        self.resolution[0] * self.resolution[1] * 3)
                    image = array.array('b', image_byte_array)
                    image.reverse()
                    im = img.frombuffer("RGB", (self.resolution[0], self.resolution[1]), image, "raw", "RGB", 0, 1)
                    imr = im.transpose(img.FLIP_TOP_BOTTOM)
                    pic = ImageTk.PhotoImage(imr)
                    self.canvas_vrep.itemconfigure(self.canvas_vrep.mypic, image=pic)

                    # Robot data
                    Robot_data, addr_odom_GUI = socket_odom_GUI.recvfrom(1024)
                    robot_x, robot_y, robot_angle = struct.unpack('3d', Robot_data)
                    self.label_posX.config(text=robot_x)
                    self.label_posY.config(text=robot_y)
                    self.label_angle.config(text=robot_angle)

                    # Draw trajectory
                    a.scatter(robot_x, robot_y, color='blue')
                    canvas.draw()

    ### START SIMULATION ###
    def Start(self):
        start = 1
        try:
            udp_data_send = struct.pack('b', start)
            socket_start.sendto(udp_data_send, addr_out)
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)
            self.isrun = True
            print ("Simulation started")

            self.label_angle = Label(root, bg="#D8E7F8")
            self.label_angle.place(x=80, y=45)
            self.label_posX = Label(root, bg="#D8E7F8")
            self.label_posX.place(x=80, y=70)
            self.label_posY = Label(root, bg="#D8E7F8")
            self.label_posY.place(x=80, y=95)

        except:
            print ("Simulation not started")

    ### STOP SIMULATION ###
    def Stop(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)
        self.isrun = False
        print ("Simulation stopped")


class Camera(Canvas):
    def __init__(self, root, size=[128, 128], path='pic1.png'):
        Canvas.__init__(self, root, height=size[0], width=size[1], bg="blue")
        self.pic = ImageTk.PhotoImage(img.open(path))
        self.mypic = self.create_image([66, 66], image=self.pic)
        self.pack()


root = Tk()
root.title("Graphical interface for DR-12")
root.geometry("640x360+250+50")
root.resizable(False, False)
myframe = Frame(root, bg="#D8E7F8")
myframe.pack(fill="both", expand=True)

CamCanvas = Camera(myframe)
CamCanvas.place(x=55, y=160)

cam = Canvas(root, width=60, height=20, bg="#D8E7F8")
cam.place(x=88, y=135)
cam.create_text(32, 10, text='Image', fill='blue', font='Arial 12')

myaction = GUI(0.5, CamCanvas)
myaction.start()

# Creating Buttons
btn_start = Button(root, text='Start', bg='white', fg='blue',
                   padx='12', pady='6', font='12', command=myaction.Start)
btn_stop = Button(root, text='Stop', bg='white', fg='red',
                  padx='12', pady='6', font='12', command=myaction.Stop)
btn_start.place(x=45, y=310)
btn_stop.place(x=130, y=310)

# Canvas for data
data = Canvas(root, width=200, height=110, bg="#D8E7F8")
data.place(x=20, y=15)
data.create_text(100, 10, text='Data of robot', fill='blue', font='Arial 12')
data.create_text(30, 40, text='Angle:', fill='black')
data.create_text(30, 65, text='Position x:', fill='black')
data.create_text(30, 90, text='Position y:', fill='black')

# Create trajectory plot
f = Figure(figsize=(3, 3), dpi=100)
a = f.add_subplot(111)
canvas = FigureCanvasTkAgg(f, root)
canvas.draw()
canvas._tkcanvas.place(x=300, y=35)

traj = Canvas(root, width=80, height=20, bg="#D8E7F8")
traj.place(x=405, y=10)
traj.create_text(40, 10, text='Trajectory', fill='blue', font='Arial 12')

root.mainloop()