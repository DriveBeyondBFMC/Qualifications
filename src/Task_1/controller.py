import numpy as np
import matplotlib.pyplot as plt
import pyclothoids


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
        return 2 * self.l * alpha / self.Ld
    
                    
class CommandSystem:
    def __init__(self, stoppingRadius = 15, switchRadius = 15):
        self.stopFlag = False
        self.switchFlag = False
        self.stoppingRadius = stoppingRadius    
        self.switchRadius = switchRadius
        self.nextMajorPoint = 1
        self.prepareStopPoint = None
        self.switchPoint = None
        self.prepareStopFlag = False
        
    def stopControl(self, currentVelocity, currentPosition, distance):
        velocityControl = currentVelocity

        if (distance[-1] <= self.stoppingRadius and self.nextMajorPoint == len(distance)) or self.prepareStopFlag:
            if self.prepareStopFlag is False:
                self.prepareStopPoint = np.array(currentPosition)
                self.prepareStopFlag = True
            else:
                if np.linalg.norm(self.prepareStopPoint - np.array([currentPosition])) >= self.stoppingRadius:
                    velocityControl = 0
                    self.stopFlag = True
            
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
        s_vals = np.linspace(0, clothoid.length, 300)
        x_segment = [clothoid.X(s) for s in s_vals]
        y_segment = [clothoid.Y(s) for s in s_vals]

        x_vals.extend(x_segment)
        y_vals.extend(y_segment)
        
    return np.array([x_vals, y_vals])
