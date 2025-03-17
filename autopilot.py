import serial
import time
import sys
import os
import threading
import socket
import re
import numpy as np
from utils.controller import *
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.bridge.connection import Sender

stateLock = threading.Lock()

serial_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)

distAvoid = [np.inf, np.inf]
collectedDynamic = False
initDynamic = [0, 0, 0]
speed_param = 0
steer_param = 0
states = [0, 0, 0]
statesUWB = [0, 0]
tempState = [0, 0, 0]
resetCoordinate = [[146.5, 230.5, 360, 0], [60, 347, 275, 15], [375, 230.5, 360, 17]]


# CMD list format
commands = {
    '0': '#kl:30;;\r\n',
    '1': '#battery:0;;\r\n',
    '2': '#instant:0;;\r\n',
    '3': '#imu:0;;\r\n',
    '4': '#resourceMonitor:0;;\r\n',
    '5': '#speed:{0};;\r\n',
    '6': '#steer:{0};;\r\n',
    '7': '#vcd:200;0;121;\r\n',
    '8': '#vehicledynamic:{0};;\r\n',
    '9': '#vehicledynamic:1;;\r\n'
}


cameraCmd = {'steer': 90, 'speed': 0, 'parkingMode': 'False', 'avoidMode': 'False', "stopLine": "False", "stopSign": "False"}
trafficCmd = {}
host = "127.0.0.1"        
parkingPort = 8002
laneGuidePort = 8001
trafficLightPort = 5007

coorSender = Sender("192.168.0.6", 5005)
velocitySender = Sender("192.168.0.106", 4000)

laneGuidanceSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
laneGuidanceSocket.bind((host, laneGuidePort))
    
parkingModeSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
parkingModeSocket.bind((host, parkingPort))

trafficLightSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
trafficLightSocket.bind(("192.168.0.4", trafficLightPort))
def send_command(command):
    ser.write(command.encode())
    # print(f"Sent: {command.strip()}")

def resetDynamic(resetCoor):
    time.sleep(3)
    global tempState, initDynamic, states
    initDynamic = tempState
    prevDynamicTime = time.time()
    for _ in range(3):
        angle = int(resetCoor[2])
        send_command(commands['8'].format(angle))
        time.sleep(0.5)
    while True:

        if time.time() - prevDynamicTime >= 1:
            initDynamic[0] = - states[0] + resetCoor[0]
            initDynamic[1] = - states[1] + resetCoor[1]
            initDynamic[2] = states[0] * 10
            break

    
        

def read_response():
    global states, statesUWB, tempState
    initUWB = []
    prevUWBTime = time.time()

    collectedUWB = False
    while True:
        time.sleep(.05)
        if ser.in_waiting > 0:  # Kiểm tra xem có dữ liệu trong buffer hay không
            response = ser.readline().decode('utf-8').strip()
            
            try:
                with stateLock:
                    if "x: " in response:
                        for idx, coorString in enumerate(response.split(", ")):
                            states[idx] = float(coorString.split(": ")[-1]) / 10
                            
                        states[2] *= 10
                            
                    if "x_uwb: " in response:
                        for idx, coorString in enumerate(response.split(", ")):
                            statesUWB[idx] = float(coorString.split(": ")[-1])
                        if time.time() - prevUWBTime <= 3:
                            initUWB.append(statesUWB[:])
                        else: collectedUWB = True
                    
                    if "distance:" in response:
                        distances = response.split(":")[-1]
                        for idx, dist in enumerate(distances.split(" ")):
                            distAvoid[idx] = int(dist)
                
                if collectedUWB:
                    avgUWB = np.mean(initUWB[:], axis = 0)
                    states[0] = states[0] + avgUWB[0]
                    states[1] = states[1] + avgUWB[1]
                
            except: ...

def readMessage(socketer):
    global cameraCmd, trafficCmd
    while True:
        data, addr = socketer.recvfrom(1024)    
        message = data.decode("utf-8")

        splitCmd = message.split("|")
        
        if addr[0] == "192.168.0.2":
            id = None; foundId = False
            for unprocessedCmd in splitCmd:
                key, value = unprocessedCmd.split(": ")
                if key == "id":
                    id = value
                    foundId = True
                    trafficCmd.update({int(id): {}})
                
                if foundId and key != 'id':
                    trafficCmd[int(id)].update({key: value})
                
        else:
            for unprocessedCmd in splitCmd:
                key, value = unprocessedCmd.split(": ")
                cameraCmd[key] = value

def unpacking():
    time.sleep(2)
    send_command(commands['5'].format(-90))
    velocitySender.sendMessage(f"{int(90)} 100")
    time.sleep(3)
    send_command(commands['5'].format(0))
    velocitySender.sendMessage(f"{int(0)} 100")
    time.sleep(2)
    send_command(commands['6'].format(-225))
    time.sleep(2)
    send_command(commands['5'].format(100))
    velocitySender.sendMessage(f"{int(100)} 100")
    time.sleep(8)
    send_command(commands['5'].format(0))
    velocitySender.sendMessage(f"{int(0)} 100")
    send_command(commands['6'].format(225))
    time.sleep(7.5)
    send_command(commands['6'].format(0))
    time.sleep(0.1)
    send_command(commands['5'].format(0))
    velocitySender.sendMessage(f"{int(0)} 100")

def avoid_object():

    send_command(commands['5'].format(120))
    velocitySender.sendMessage(f"{int(120)} 100")
    time.sleep(0.5)
    send_command(commands['6'].format(-200))
    time.sleep(4)
    send_command(commands['6'].format(170))
    time.sleep(3)
    while True:
        angle = float(cameraCmd['steer'])
        steer_param = int((angle / 30) * 230)
        if steer_param > 230: steer_param = 230
        if steer_param < -230: steer_param = -230
        send_command(commands['6'].format(steer_param))
        time.sleep(.02)
        send_command(commands['5'].format(140))
        velocitySender.sendMessage(f"{int(140)} 100")
        time.sleep(.02)
        if distAvoid[1] > 500: 
            break
    send_command(commands['6'].format(225))
    time.sleep(0.1)
    send_command(commands['5'].format(140))
    velocitySender.sendMessage(f"{int(140)} 100")
    time.sleep(3.5)
    send_command(commands['5'].format(0))
    velocitySender.sendMessage(f"{int(0)} 100")
    time.sleep(0.1)
    send_command(commands['6'].format(-225))
    time.sleep(0.1)
    send_command(commands['5'].format(140))
    velocitySender.sendMessage(f"{int(140)} 100")
    time.sleep(1.5)

def parking():

    send_command(commands['6'].format(230))
    time.sleep(2)
    send_command(commands['5'].format(-90))
    velocitySender.sendMessage(f"{int(90)} 100")
    time.sleep(7.8)
    send_command(commands['6'].format(-230))
    time.sleep(2)
    send_command(commands['5'].format(-90))
    velocitySender.sendMessage(f"{int(90)} 100")
    time.sleep(5.3)
    send_command(commands['6'].format(50))
    time.sleep(0.1)
    send_command(commands['5'].format(100))
    velocitySender.sendMessage(f"{int(100)} 100")
    time.sleep(3.5)
    send_command(commands['5'].format(0))
    velocitySender.sendMessage(f"{int(0)} 100")
    time.sleep(0.1)
    send_command(commands['6'].format(0))
    
def readPath(filename: str):
    data_arrays = {}
    with open(filename, "r") as file:
        for line in file:
            key, values = line.split(":")
            # Remove brackets and commas, then split into a list
            cleaned_values = re.sub(r"[\[\],]", "", values).strip()
            values_list = np.array([float(v) for v in cleaned_values.split()])
            data_arrays[key.strip()] = values_list

    # Accessing the arrays
    yPath = data_arrays["Array_y"] 
    xPath = data_arrays["Array_x"] 
    theta = np.pi * 2 - (data_arrays["Array_z"] + np.pi / 2)
    cameraSwitch = data_arrays["Array_t"]
    
    mask = np.insert((np.diff(xPath) != 0) | (np.diff(yPath) != 0), 0, True)

    xPath = xPath[mask]
    yPath = yPath[mask]
    theta = theta[mask]
    t = cameraSwitch[mask]

    return xPath, yPath, theta, t
    
    

def main():
    global speed_param, steer_param
    

    xPath = [
        0, 240, 380, 530, 647, 647 
    ]

    yPath = [
        0, 0, 0, 0, 105, 150
    ]


    theta = [0, 0, 0, 0, np.pi / 2, np.pi / 2]

    # t = [0, 1, 0, 1, 0, 0]    
    t = [0, 0, 0, 0, 0, 0]    
    xPath, yPath, theta, t = readPath("./plannedCoor.txt")
    definedPath = clothoidsGen(xPath, yPath, theta)

    referenceVelocity = 17
    l = 26
    Ld = 50
    controller = PDPPController(Ld, l)
    lheFinder = LHEFinder(Ld, definedPath)
    commandCenter = CommandSystem(25, 50)

    prepRun = time.time()
    cntStopSign = 0; prevStopSignCnt = 0;
    resetDynamic(resetCoordinate[0])
    try:
        parkingMode = False; angle = 0; speed_param = referenceVelocity * 10; avoidMode = False
        time.sleep(.5)
        # with open("CoordinateLog.txt", "w") as f:
        while True:
            with stateLock:
                vehicleX = states[0] + initDynamic[0]
                vehicleY = states[1] + initDynamic[1]
                psi = states[2] * np.pi / 180
            

            distance = distance2Path(vehicleX, vehicleY, definedPath)
            distance2MajorPoint = np.r_[distance[::300], distance[-1]] # change this shit
            lheFinder.updateState(distance)
            
            # velocity = commandCenter.stopControl(referenceVelocity, [vehicleX, vehicleY], distance2MajorPoint)
            # commandCenter.switchControl([vehicleX, vehicleY], distance2MajorPoint)
            velocity = commandCenter.stopControl(referenceVelocity, definedPath[:, np.argmin(distance)], distance2MajorPoint)
            commandCenter.switchControl(definedPath[:, np.argmin(distance)], distance2MajorPoint)
            
            if commandCenter.nextMajorPoint < len(t) and t[commandCenter.nextMajorPoint - 1]:
                alpha = lheFinder.LHE(vehicleX, vehicleY, psi, definedPath)
                angle = controller.PDcontrol(alpha, .9, 0)
                angle = np.clip(angle * 180 / np.pi, -30, 30)
                steer_param = -(int(angle) / 30) * 230            
                print("Dynamic")
            else:
                parkingMode = False if cameraCmd["parkingMode"] == "False" else True
                avoidMode = False if cameraCmd["avoidMode"] == "False" else True
                velocity = float(cameraCmd['speed'])
                angle = float(cameraCmd['steer'])
                steer_param = int((angle / 30) * 230)
                if steer_param > 230: steer_param = 230
                if steer_param < -230: steer_param = -230
                print("Camera")
            
            
            # check for intersection
            if commandCenter.nextMajorPoint < len(t) and (t[commandCenter.nextMajorPoint - 1] == False and t[commandCenter.nextMajorPoint] == True) and distance2MajorPoint[commandCenter.nextMajorPoint] < 50: 
                for id, state in trafficCmd.items():
                    trafficLightCoor = np.array([float(state['x']), float(state['y'])])
                    trafficLightState = int(state['state'])
                
                    dist2StopLight = np.linalg.norm(trafficLightCoor - np.array([vehicleX, vehicleY]))
                    print(dist2StopLight, trafficLightState, cameraCmd['stopLine'])
                    if dist2StopLight < 60: velocity = 12
                    if dist2StopLight < 60 and (trafficLightState == 0 or trafficLightState == 1) and cameraCmd['stopLine'] == 'True':
                            velocity = 0

            speed_param = velocity * 10
            
            # Reset point
            if cameraCmd['stopSign'] == 'True' and foundStopSign == False:
                cntStopSign += 1
                foundStopSign = True 
            elif cameraCmd['stopSign'] == 'False': foundStopSign = False
            
            print([vehicleX, vehicleY, states[2]], lheFinder.currLookahead,  velocity, cameraCmd['stopSign'], cntStopSign, commandCenter.nextMajorPoint - 1)
            send_command(commands['6'].format(steer_param))
            if time.time() - prepRun > 7:
                time.sleep(.02)
                velocitySender.sendMessage(f"{int(speed_param)} 100")
                send_command(commands['5'].format(speed_param))
                ...
            time.sleep(.02)

            if cntStopSign >= 2 and prevStopSignCnt != cntStopSign:
                speed_param = 0
                velocitySender.sendMessage(f"{int(speed_param)} 100")
                send_command(commands['5'].format(0))
                prevStopSignCnt = cntStopSign
                # reset System
                commandCenter.nextMajorPoint = resetCoordinate[cntStopSign - 1][3]
                resetDynamic(resetCoordinate[cntStopSign - 1])

            coorSender.sendMessage(f"x: {definedPath[0, np.argmin(distance)]}| y: {definedPath[1, np.argmin(distance)]}")
            if avoidMode:
                avoid_object()
                send_command(commands['5'].format(0))
                
            if parkingMode == True:                 
                send_command(commands['5'].format(speed_param))
                time.sleep(.5)
                send_command(commands['5'].format(0))
                time.sleep(2)
                parking()
                time.sleep(2)
                unpacking()

    except KeyboardInterrupt:
        ...

if __name__ == "__main__":
    # Tạo một thread cho việc đọc dữ liệu từ STM32
    serial_thread = threading.Thread(target=read_response)
    thread1 = threading.Thread(target = readMessage, args = (laneGuidanceSocket, ))
    thread2 = threading.Thread(target = readMessage, args = (parkingModeSocket, ))
    thread3 = threading.Thread(target = readMessage, args = (trafficLightSocket, ))
    thread1.start()
    thread2.start()
    thread3.start()
    serial_thread.daemon = True  # Đảm bảo rằng thread này sẽ tự động kết thúc khi chương trình chính kết thúc
    serial_thread.start()

    # Chạy chương trình chính
    main()

    # Đảm bảo đóng cổng serial khi kết thúc chương trình
    ser.close()
