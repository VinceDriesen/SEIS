from controller import Motor, Robot, PositionSensor, DistanceSensor
import math

class TurtleBot:
    def __init__(self, robot: Robot, timeStep: int, maxSpeed: float):
        self.robot = robot
        self.timeStep = timeStep
        self.maxSpeed = maxSpeed
        self.leftMotor: Motor = self.robot.getDevice('left wheel motor')
        self.rightMotor: Motor = self.robot.getDevice('right wheel motor')
        self.enableSensors()
  

    def enableSensors(self):
        self.frontDistSens: DistanceSensor = self.robot.getDevice('front distance sensor')
        self.rearDistSens: DistanceSensor = self.robot.getDevice('rear distance sensor')
        self.leftDistSens: DistanceSensor = self.robot.getDevice('left distance sensor')
        self.rightDistSens: DistanceSensor = self.robot.getDevice('right distance sensor')
        self.leftMotorSens: PositionSensor = self.robot.getDevice('left wheel sensor')
        self.rightMotorSens: PositionSensor = self.robot.getDevice('right wheel sensor')
        
        self.frontDistSens.enable(self.timeStep)
        self.rearDistSens.enable(self.timeStep)
        self.leftDistSens.enable(self.timeStep)
        self.rightDistSens.enable(self.timeStep)
        self.leftMotorSens.enable(self.timeStep)
        self.rightMotorSens.enable(self.timeStep)
      
    def turn(self, degrees: float):
        leftMotor = self.leftMotor
        rightMotor = self.rightMotor
        leftMotorSens = self.leftMotorSens
        rightMotorSens = self.rightMotorSens

        desiredRadians = math.radians(degrees)

        wheelRadius = 0.02
        wheelbase = 0.1

        wheelRotation = (wheelbase / (2 * wheelRadius)) * desiredRadians

        currentLeftRotation = leftMotorSens.getValue()
        currentRightRotation = rightMotorSens.getValue()

        newLeftRotation = currentLeftRotation + wheelRotation
        newRightRotation = currentRightRotation - wheelRotation

        leftMotor.setPosition(newLeftRotation)
        rightMotor.setPosition(newRightRotation)
        leftMotor.setVelocity(self.maxSpeed / 2)
        rightMotor.setVelocity(self.maxSpeed / 2)

        while self.robot.step(self.timeStep) != -1:
            leftCurrent = leftMotorSens.getValue()
            rightCurrent = rightMotorSens.getValue()
            if abs(leftCurrent - newLeftRotation) < 0.01 and abs(rightCurrent - newRightRotation) < 0.01:
                break
