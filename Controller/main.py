import json

from controller import Robot, Motor, DistanceSensor
from src.turtleBot import TurtleBot

TIME_STEP = 64
MAX_SPEED = 6.28

def serialize(obj):
    """Convert non-serializable objects into dictionaries."""
    if hasattr(obj, '__dict__'):
        return obj.__dict__  # Convert objects with __dict__ to dictionaries
    return str(obj)  # Convert unknown objects to strings


def main():
    robot = Robot()
    
    print(json.dumps(robot.devices, indent=4, default=serialize))
    
    bot = TurtleBot(robot, TIME_STEP, MAX_SPEED)
    
    test = False
    while robot.step(TIME_STEP) != -1:
        if test is False:
            bot.turn(127)
            test = True
        pass
    
    
# def main():
#     # Create the Robot instance.
#     robot = Robot()
#     # Print robot components
#     print(json.dumps(robot.devices, indent=4, default=serialize))
    
#     # Get Devices
#     leftMotor: Motor = robot.getDevice('left wheel motor')
#     rightMotor: Motor = robot.getDevice('right wheel motor')
#     frontDistSens: DistanceSensor = robot.getDevice('front distance sensor')
#     rearDistSens: DistanceSensor = robot.getDevice('rear distance sensor')
#     leftDistSens: DistanceSensor = robot.getDevice('left distance sensor')
#     rightDistSens: DistanceSensor = robot.getDevice('right distance sensor')
    
#     #Enable Sensors
#     frontDistSens.enable(TIME_STEP)
#     rearDistSens.enable(TIME_STEP)
#     leftDistSens.enable(TIME_STEP)
#     rightDistSens.enable(TIME_STEP)
    
#     #Set Motor positions to infintie
#     leftMotor.setPosition(float('inf'))
#     rightMotor.setPosition(float('inf'))
    
#     #Set Motor Velocitity
#     leftMotor.setVelocity(0.1 * MAX_SPEED)
#     rightMotor.setVelocity(0.1 * MAX_SPEED)

#     print("Connected to Webots simulation. Robot name:", robot.getName())

#     movingForward = True
    
#     # Main control loop
        
        
#         frontDistSensVal = frontDistSens.getValue()
#         rearDistSensVal = rearDistSens.getValue()           
        
#         if(frontDistSensVal >= 900 and movingForward):
#             leftMotor.setVelocity(-0.1 * MAX_SPEED)
#             rightMotor.setVelocity(-0.1 * MAX_SPEED)
#             movingForward = False
        
#         if(rearDistSensVal >= 900 and not movingForward):
#             leftMotor.setVelocity(0.1 * MAX_SPEED)
#             rightMotor.setVelocity(0.1 * MAX_SPEED)
#             movingForward = True
        

if __name__ == '__main__':
    main()