###############################################################
###############################################################
# In this code you can find out the following (for controlling, commanding and measuring the dynamixel):
# 1. How to use the dynamixel library to control a single motor in many operational modes
# 2. How to use the dynamixel library to read state of a single motor
# 3. Saving real-time dynamixel data and plottiing the trajectory of a single motor and the load of the motor
# *************************************************************
# *************************************************************
# This code is written for Python3 by M. J. Zallaghi
# You can contact me at
# My E-Mail: MohammadJavadZallaghi@gmail.com
# My Github: https://github.com/MJavadZallaghi
# My LinkedIn: https://www.linkedin.com/in/mohammad-javad-zallaghi-061645106/
###############################################################
###############################################################

from os import error, path, sep
from threading import current_thread
import serial
import keyboard
import matplotlib.pyplot as plt
plt.rcParams['animation.html'] = 'jshtml'
import time
import numpy as np
import math
import pandas as pd
from timeit import default_timer as timer
# imports For MX-64(2) Motor Controll
import os
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
# MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V
# ADDR of MX-64(2) Motor Controll (for other series use other ADDR from dynamixel site control table)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
ADDR_GOAL_VELOCITY          = 104
LEN_GOAL_VELOCITY           = 4
ADDR_GOAL_CURRENT           = 102
LEN_GOAL_CURRENT            = 2
ADDR_PRESENT_CURRENT         = 126
LEN_PRESENT_CURRENT          = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4 
DXL_MINIMUM_VELOCITY_VALUE = -100 # -1023 to 1023
DXL_MAXIMUM_VELOCITY_VALUE = 100
ADDR_OPERATING_MODE = 11
LEN_OPERATING_MODE = 1
CUURENT_CONTROL_MODE = 0
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
CURRENT_LIMIT = [0,1941] # Unit: 3.36 mA = maimum current in A is 6.52 A
VELOCITY_LIMIT = [0,1023] # Unit: 0.229 rpm - maximum speed in rpm is 234.267
POSITION_LIMIT = [0,4095] # Unit: 0.088 degree
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val  
def speedInRPMforDXL(speed):
    if speed>=0:
        sign = 1
    else:
        sign = -1
    target = int(abs(speed/0.229))
    if target > VELOCITY_LIMIT[1]:
        target = VELOCITY_LIMIT[1]
    return sign*target
def speedInRPMfromDXL(speed):
    speed = twos_comp(speed, LEN_PRESENT_VELOCITY*8)
    if speed >=0 and speed <= VELOCITY_LIMIT[1]:
        target = speed * 0.229
    elif speed <0 and speed >= -1023:
        target = speed * 0.229
    elif speed > VELOCITY_LIMIT[1]:
        target = VELOCITY_LIMIT[1] * 0.229
    elif speed < -VELOCITY_LIMIT[1]:
        target = -VELOCITY_LIMIT[1] * 0.229
    return target
def posInDegfromDXL(pos):
    pos = twos_comp(pos, LEN_PRESENT_POSITION*8)
    return pos * 0.088
def posInDegforDXL(pos):
    target = int((pos/360)*4095)
    return pos * 0.088
def curInAforDXL(cur):
    try:
        if cur >=0:
            sign = 1
        else:
            sign = -1
        target = int(abs(cur*1000/3.36))
        if target > CURRENT_LIMIT[1]:
            target = CURRENT_LIMIT[1]
        return sign*target
    except:
        return 0
def curInAfromDXL(cur):
    cur = twos_comp(cur, LEN_PRESENT_CURRENT*8)
    if cur >=0 and cur <= CURRENT_LIMIT[1]:
        target = cur * (3.36/1000)
    elif cur <0 and cur >= -CURRENT_LIMIT[1]:
        target = cur * (3.36/1000)
    elif cur > CURRENT_LIMIT[1]:
        target = CURRENT_LIMIT[1] * (3.36/1000)
    elif cur < -1023:
        target = -CURRENT_LIMIT[1] * (3.36/1000)
    return target
index = 2
dxl_goal_velocity = [DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE]  # Goal velocity
dxl_goal_velocity = [-255,0,255]
# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0
# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 3                 # Dynamixel#1 ID : 1
# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM6'
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupSyncWrite instance
groupSyncWritePos = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncWriteVel = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
groupSyncWriteCur = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)
# Initialize GroupSyncRead instace for Present Position
groupSyncReadPos = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
groupSyncReadCur = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
groupSyncReadVel = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
def torqueEnaDis(actiavte):
    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, actiavte)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque has been changed - " % DXL1_ID, str(actiavte))
def setOperatingMode(mode):
    torqueEnaDis(TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d mode has been changed - " % DXL1_ID, str(mode))
    torqueEnaDis(TORQUE_ENABLE)
dxl_addparam_result = groupSyncReadPos.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncReadPos addparam failed" % DXL1_ID)
    quit()
# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncReadCur.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncReadCur addparam failed" % DXL1_ID)
    quit()
# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncReadVel.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncReadVel addparam failed" % DXL1_ID)
    quit()
def speedCommand(speedm1):
    # Allocate goal position value into byte array
    param_goal_velocity_m1 = [DXL_LOBYTE(DXL_LOWORD(speedm1)), DXL_HIBYTE(DXL_LOWORD(speedm1)), DXL_LOBYTE(DXL_HIWORD(speedm1)), DXL_HIBYTE(DXL_HIWORD(speedm1))]
    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteVel.addParam(DXL1_ID, param_goal_velocity_m1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWriteVel.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWriteVel.clearParam()
def posCommand(pos1):
    # Allocate goal position value into byte array
    param_goal_pos_m1 = [DXL_LOBYTE(DXL_LOWORD(pos1)), DXL_HIBYTE(DXL_LOWORD(pos1)), DXL_LOBYTE(DXL_HIWORD(pos1)), DXL_HIBYTE(DXL_HIWORD(pos1))]
    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWritePos.addParam(DXL1_ID, param_goal_pos_m1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWritePos.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWritePos.clearParam()
def currentCommand(currentm1):
    param_goal_current_m1 = [DXL_LOBYTE(DXL_LOWORD(currentm1)), DXL_HIBYTE(DXL_LOWORD(currentm1))]
    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteCur.addParam(DXL1_ID, param_goal_current_m1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWriteCur.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWriteCur.clearParam()
def getPosLoadMotorData():
    # Syncread present position
    dxl_comm_result = groupSyncReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncReadPos.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncReadPos getdata failed" % DXL1_ID)
        quit()
    # Syncread present position
    dxl_comm_result = groupSyncReadCur.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncReadCur.isAvailable(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncReadCur getdata failed" % DXL1_ID)
        quit()
    # Syncread present position
    dxl_comm_result = groupSyncReadVel.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncReadVel.isAvailable(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncReadVel getdata failed" % DXL1_ID)
        quit()
    # Get Dynamixel#1 present position value
    dxl1_present_position = groupSyncReadPos.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    # Get Dynamixel#1 load value
    dxl1_present_load = groupSyncReadCur.getData(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
    # Get Dynamixel#1 present velocity value
    dxl1_present_velocity = groupSyncReadVel.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
    # return data
    # print([dxl1_present_position, dxl2_present_position, dxl1_present_load, dxl2_present_load, dxl1_present_velocity, dxl2_present_velocity])
    return [dxl1_present_position, dxl1_present_load, dxl1_present_velocity]
def ForceControl(des, force, k, error_bound=0.5):
    # PID Force Contrrol Law (Constants Desigred Inputed)
    kp = k[0]
    kd = k[1]
    ki = k[2]
    error1 = des- force[-1]
    error1d = 0 - (force[-1]-force[-2])
    errorI1 = des - sum(force)
    if abs(error1) > error_bound:
        z1 = int(kp*error1+kd*error1d+ki*errorI1)
    else:
        z1 = 0
    currentCommand(curInAforDXL(z1))
def PosControl(des, pos, k, error_bound=1):
    # PID Force Contrrol Law (Constants Desigred Inputed)
    kp = k[0]
    kd = k[1]
    ki = k[2]
    error1 = des - pos[-1]
    error1d = 0 - (pos[-1]-pos[-2])
    # it is not corerect but we are using it for now
    errorI1 = des[0] - sum(pos[0])
    if abs(error1) > error_bound:
        z1 = int(kp*error1+kd*error1d+ki*errorI1)
    else:
        z1 = 0
    currentCommand(curInAforDXL(z1))
def harmonicTorqueGenerator(constant, amplitude, omega, phase, t):
    return (constant+ amplitude*math.sin(omega*t+phase))

def harmonic_trajecory(constant, amplitude, omega, phase, t):
    return constant + amplitude*np.sin(omega*t+phase)
# setOperatingMode(CUURENT_CONTROL_MODE)
setOperatingMode(VELOCITY_CONTROL_MODE)
# time reading
start = timer()
# speed in rpm
speed = 60
# storing motor data
m1theta = []
m1omega = []
m1load = []
timeArray = []
while True:
    try:
        dynamixel_curren_data = getPosLoadMotorData()
        m1theta.append(dynamixel_curren_data[0])
        m1load.append(curInAfromDXL(dynamixel_curren_data[2]))
        m1omega.append(speedInRPMfromDXL(dynamixel_curren_data[4]))
        timeArray.append(timer())
    except:
        print('data is NOT available')
        continue
    # break loop by krboard input
    if keyboard.is_pressed('q'):
        # currentCommand(0)
        speedCommand(0)
        # posCommand(0)
        break
    elif keyboard.is_pressed('f'):
        # currentCommand(curInAforDXL(harmonicTorqueGenerator(0, 0.1, 0.1, timeArray[-1])))
        # currentCommand(20)
        speedCommand(speedInRPMforDXL(speed))
        # posCommand(4095)
    elif keyboard.is_pressed('b'):
        # currentCommand(-25)
        speedCommand(speedInRPMforDXL(-speed))
        # posCommand(0)
        pass
m1theta = [(theta - m1theta[0])*(360/POSITION_LIMIT[1])*(np.pi/180) for theta in m1theta]
timeArray = [t - timeArray[0] for t in timeArray]
# Storing data in a panda data frame
array_min_length = min(len(timeArray), len(m1theta), len(m1load))
# create a data frame in pandas
df = pd.DataFrame({'time': timeArray[:array_min_length], 'theta': m1theta[:array_min_length], 'load': m1load[:array_min_length], 'omega': m1omega[:array_min_length]})
# add time list to df 'time'
df['time'] = timeArray[0:array_min_length]
# add m1theta list to df 'mte1'
df['mte1'] = m1theta[0:array_min_length]
# add m1vel list to df 'mo1'
df['mo1'] = m1omega[0:array_min_length]
# add m1load list to df 'ml1'
df['ml1'] = m1load[0:array_min_length]
# save the data frame to excel
path = "D:\\Javad\\Master's project\\codes\\Arduino\\data" # change it to your own desired path
current_date = str(pd.datetime.now().date())+'-'+ str(pd.datetime.now().time()).split('.')[0].split(':')[0]+'-'+str(pd.datetime.now().time()).split('.')[0].split(':')[1]+'-'+str(pd.datetime.now().time()).split('.')[0].split(':')[2]
data_save_name_xlsx = current_date + '.xlsx'
data_save_name_png_realTime = current_date + 'RealTime' + '.png'
data_save_name_png_Final = current_date + '.png'
df.to_excel(r'data\\'+data_save_name_xlsx)
# Plot motor data
def FinalTimePlot():
    fig, ax = plt.subplots(3, 1)
    ax[1, 0].plot(timeArray[0:array_min_length], m1theta[0:array_min_length], 'r')
    ax[1, 0].set_ylabel('Motor Angle (Rad)')
    ax[2, 0].plot(timeArray[0:array_min_length], m1omega[0:array_min_length], 'r')
    ax[2, 0].set_ylabel('Motor Velocity (RPM)')
    ax[3, 0].plot(timeArray[0:array_min_length], m1load[0:array_min_length], 'r',)
    ax[3, 0].set_xlabel('Time')
    ax[3, 0].set_ylabel('Motor Load (A)')
    plt.show()
    fig.savefig('data\\'+data_save_name_png_Final)
FinalTimePlot()
