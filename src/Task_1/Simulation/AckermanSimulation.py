import numpy as np
import matplotlib.pyplot as plt
import pyclothoids
from tqdm import tqdm
import re


class VehicleDynamic:
    def __init__(self, x = 0, y = 0, psi = 0, L = 26):
        self.x = x
        self.y = y
        self.psi = psi # heading angle
        self.L = L

    def updateRule(self, v, phi, psi):
        xdot = v * np.cos(psi)
        ydot = v * np.sin(psi)
        psidot = v * np.tan(phi) / self.L
        
        return xdot, ydot, psidot
    
    def updateState(self, v, phi, dt = 0.01):
        x1 = [v, phi, self.psi]
        f1 = self.updateRule(*x1)
        
        x2 = [v, phi, self.psi + f1[2] * dt / 2]
        f2 = self.updateRule(*x2)   
        
        x3 = [v, phi, self.psi + f2[2] * dt / 2]
        f3 = self.updateRule(*x3)
        
        x4 = [v, phi, self.psi + f3[2] * dt]
        f4 = self.updateRule(*x4)
        
        self.x += dt / 6 * (f1[0] + 2*f2[0] + 2*f3[0] + f4[0])
        self.y += dt / 6 * (f1[1] + 2*f2[1] + 2*f3[1] + f4[1])
        self.psi += dt / 6 * (f1[2] + 2*f2[2] + 2*f3[2] + f4[2])

class Derivative:
    def __init__(self, alpha = 0.9):
        self.prevU = None
        self.derivative = 0.0
        self.alpha = alpha

    def updateState(self, u, dt = 0.01):
        if self.prevU is None:
            self.prevU = u
            return self.derivative
        
        raw_derivative = (u - self.prevU) / dt
        self.prevU = u
        
        self.derivative = (self.alpha * self.derivative
                           + (1.0 - self.alpha) * raw_derivative)
        return self.derivative
        
    
class LHEFinder:
    def __init__(self, Ld, path):
        self.Ld = Ld
        self.currLookahead = 0
        self.allPathIdx = np.arange(len(path))
        
    def updateState(self, distance):
        inRangePathIdx = np.where(np.abs(distance - self.Ld) <= self.Ld)[0]
        splitIndices = np.where(np.diff(inRangePathIdx) != 1)[0] + 1
        consecutiveGroups = np.split(inRangePathIdx, splitIndices)

        for groupIndices in consecutiveGroups:
            if self.currLookahead in groupIndices:
                minIdxIngroup = np.argmin(np.abs(distance[groupIndices] - self.Ld))
                self.currLookahead = groupIndices[minIdxIngroup] if groupIndices[minIdxIngroup] > self.currLookahead else self.currLookahead
                break
        
    def LHE(self, xCurr, yCurr, psiCurr, path):
        dy = path[1, self.currLookahead] - yCurr
        dx = path[0, self.currLookahead] - xCurr
        
        
        rotMat = np.array([[np.cos(psiCurr), np.sin(psiCurr)],
                           [-np.sin(psiCurr), np.cos(psiCurr)]])

        rotatedPointDiff = rotMat @ np.array([dx, dy])
        return np.arctan2(rotatedPointDiff[1], rotatedPointDiff[0])
    
class PDPPController(Derivative):
    def __init__(self, Ld, l, dt = 1):
        super().__init__()
        self.dt = dt
        self.Ld = Ld
        self.l = l
    
    def PDcontrol(self, alpha, Kp, Kd):
        psi = self.purePursuit(alpha)
        derivative = self.updateState(alpha, self.dt)
        return Kp * psi + Kd * derivative
        
      
    def purePursuit(self, alpha):
        return 2 * self.l * np.sin(alpha) / self.Ld
    
class CommandSystem:
    def __init__(self, stoppingRadius = 15, switchRadius = 15):
        self.stopFlag = False
        self.switchFlag = False
        self.stoppingRadius = stoppingRadius    
        self.switchRadius = switchRadius
        self.nextMajorPoint = 1
        self.prepareStopPoint = None
        self.switchPoint = None
        
    def stopControl(self, currentVelocity, currentPosition, distance):
        velocityControl = currentVelocity

        if (distance[-1] <= self.stoppingRadius and self.nextMajorPoint == len(distance)) or self.stopFlag:
            if self.stopFlag is False:
                self.prepareStopPoint = np.array(currentPosition)
                self.stopFlag = True
            else:
                if np.linalg.norm(self.prepareStopPoint - np.array([currentPosition])) >= 0:
                    velocityControl = 0
            
        return velocityControl

    def switchControl(self, currentPosition, distance):
        if len(distance) > self.nextMajorPoint and (distance[self.nextMajorPoint] < self.switchRadius or self.switchFlag):
            if self.switchFlag is False:
                self.switchPoint = np.array(currentPosition)
                self.switchFlag = True
            elif self.switchFlag == True:
                if np.linalg.norm(np.array(currentPosition) - self.switchPoint) >= self.switchRadius:
                    self.nextMajorPoint += 1
                    self.switchFlag = False
                    

            
def distance2Path(xCurr, yCurr, path):
    return np.linalg.norm(np.array([xCurr, yCurr])[..., None] - path, axis = 0)


def clothoidsGen(x, y, theta):
    
    x_vals, y_vals = [], []
    for i in range(min(len(theta), len(x), len(y)) - 1):
        clothoid = pyclothoids.Clothoid.G1Hermite(x[i], y[i], theta[i], x[i+1], y[i+1], theta[i+1])
        
        # Sample points along the clothoid    pip install pyclothoids
        s_vals = np.linspace(0, clothoid.length, 50)
        x_segment = [clothoid.X(s) for s in s_vals]
        y_segment = [clothoid.Y(s) for s in s_vals]

        x_vals.extend(x_segment)
        y_vals.extend(y_segment)
        
    return np.array([x_vals, y_vals])

        
if __name__ == "__main__":

    data_arrays = {}
    with open("./plannedCoor.txt", "r") as file:
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
    cameraSwitch = cameraSwitch[mask]

    definedPath = clothoidsGen(xPath, yPath, theta)
    dt = 0.01
    time = 250
    Ld = 25
    l = 26

    UWBrefreshRate = .1 # Hz
    UWBTickRate = (1 / UWBrefreshRate) / dt
    psi = 0
    referenceVelocity = 17
    x = []
    y = []
    ey = []
    psies = []
    lhe = []
    xNoises, yNoises = [], []
    xNoisesCamera, yNoisesCamera = [], []
    prevSwitch = False
    majorPointCoorGlobal = []
    
    
    vehicle = VehicleDynamic(L = l, x = 0, y = 0, psi = 0)
    innerDynamic = VehicleDynamic(L = l, x = 0, y = 0, psi = 270 / 180 * np.pi)
    controller = PDPPController(Ld, l)
    lheFinder = LHEFinder(Ld, definedPath)
    commandCenter = CommandSystem(50, Ld)

    for currentTime in tqdm(range(int(time / dt))):
        
        
        distance = distance2Path(vehicle.x, vehicle.y, definedPath) # Actual vehicle position
        
        # UWBq noise simulation
        noisyDistance = distance2Path(innerDynamic.x,innerDynamic.y, definedPath)
        distance2MajorPoint = np.r_[noisyDistance[::50], noisyDistance[-1]]
        lheFinder.updateState(noisyDistance)
        alpha = lheFinder.LHE(innerDynamic.x, innerDynamic.y, innerDynamic.psi, definedPath)
        
        controller.dt = dt
        psi = controller.PDcontrol(alpha, 1, 0)
        psi = np.clip(psi, - 30 * np.pi / 180, 30 * np.pi / 180) # Saturation
        
        velocity = commandCenter.stopControl(referenceVelocity, definedPath[:, np.argmin(distance)], distance2MajorPoint)
        commandCenter.switchControl(definedPath[:, np.argmin(distance)], distance2MajorPoint)
        
        if commandCenter.switchFlag == False and prevSwitch == True:
            majorPointCoorGlobal.append([innerDynamic.x, innerDynamic.y])   
        prevSwitch = commandCenter.switchFlag

        vehicle.updateState(velocity, psi, dt = dt)
        if currentTime % UWBTickRate == 0:
            xNoise = np.random.normal(0, 3)
            yNoise = np.random.normal(0, 3)
            innerDynamic.x = vehicle.x + xNoise
            innerDynamic.y = vehicle.y + yNoise
        innerDynamic.psi = vehicle.psi
        innerDynamic.updateState(velocity, psi, dt = dt)

        x.append(vehicle.x)
        y.append(vehicle.y)
        ey += [np.min(distance)]
        psies += [psi * 180 / np.pi]
        lhe += [alpha * 180 / np.pi]
        if commandCenter.nextMajorPoint < len(cameraSwitch) and cameraSwitch[commandCenter.nextMajorPoint - 1]:
            xNoises.append(innerDynamic.x)  
            yNoises.append(innerDynamic.y)
        else:
            xNoisesCamera.append(innerDynamic.x)
            yNoisesCamera.append(innerDynamic.y)
        
    print(f"Final X: {vehicle.x:.2f}, Y: {vehicle.y:.2f}, Psi: {(vehicle.psi % (np.pi * 2) * 180 / np.pi):.2f}")
    print(commandCenter.nextMajorPoint - 1)
        
    fig, ax = plt.subplots(2, 2, figsize=(16, 10))

    
    # Plot vehicle trajectory
    ax[0, 0].plot(definedPath[0], definedPath[1], "r--", label="Defined Path")
    ax[0, 0].plot(x[0], y[0], "ko", label="Start Point")
    ax[0, 0].scatter(xPath, yPath)
    ax[0, 0].plot(x, y, linewidth = 0.5, label = "Vehicle Path", c = "b")
    ax[0, 0].set_xlabel("X (cm)")
    ax[0, 0].set_ylabel("Y (cm)")
    ax[0, 0].grid()
    ax[0, 0].set_box_aspect(1)  
    ax[0, 0].set_title("Vehicle Path Tracking")
    ax[0, 0].legend()

    # Plot minimum distance to path
    ax[1, 1].plot(np.arange(len(ey)) * dt, ey, "g", label="Lat. Error")
    ax[1, 1].set_xlabel("Time (s)")
    ax[1, 1].set_ylabel("Distance (cm)")
    ax[1, 1].grid()
    ax[1, 1].set_title("Minimum Distance to Path Over Time")
    ax[1, 1].legend()
    
    ax[1, 0].plot(np.arange(len(psies)) * dt, psies, "r")
    ax[1, 0].set_xlabel("Time (s)")
    ax[1, 0].set_ylabel("Degree")
    ax[1, 0].grid()
    ax[1, 0].set_title("Steering Angle Over Time")

    ax[0, 1].scatter(np.array(majorPointCoorGlobal)[:, 0], np.array(majorPointCoorGlobal)[:, 1], c = "r", s = 20)
    ax[0, 1].scatter(xNoises, yNoises, s = .5, label = "Run by UWB")
    ax[0, 1].scatter(xNoisesCamera, yNoisesCamera, s = .5, label = "Run by Camera", c = 'orange')
    ax[0, 1].set_xlabel("X (cm)")
    ax[0, 1].set_ylabel("Y (cm)")
    ax[0, 1].grid()
    ax[0, 1].set_box_aspect(1)  
    ax[0, 1].set_title("Globalisation")
    ax[0, 1].legend()

    plt.tight_layout()
    plt.show()