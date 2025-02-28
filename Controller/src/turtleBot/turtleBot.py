from controller import Motor, Robot, PositionSensor, DistanceSensor
import math

class TurtleBot: 
    def __init__(self, robot: Robot, timeStep: int, maxSpeed: float):
        self.robot = robot
        self.timeStep = timeStep
        self.maxSpeed = maxSpeed
        self.leftMotor: Motor = self.robot.getDevice('left wheel motor')
        self.rightMotor: Motor = self.robot.getDevice('right wheel motor')
        self.radius = 0.033
        self.enableSensors()
        
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
  

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


    def walkDistance(self, targetDistance: int, speedMultipleyer: float):
        velocity = self.radius * speedMultipleyer * self.maxSpeed
        duration = targetDistance / velocity
        startTime = self.robot.getTime()
        
        while self.robot.step(self.timeStep) != -1:
            currentTime = self.robot.getTime()
            print("currentTime: ", currentTime, " duration: ", duration, " startTime ", startTime)
            if currentTime >= duration + startTime:
                break
            self.leftMotor.setVelocity(speedMultipleyer * self.maxSpeed)
            self.rightMotor.setVelocity(speedMultipleyer * self.maxSpeed)
         
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        
      
    def turn(self, degrees: float, speedMultipleyer: float):
        pass