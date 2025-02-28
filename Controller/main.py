from controller import Robot, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

def main():
    # Create the Robot instance.
    robot = Robot()
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')

    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    leftMotor.setVelocity(0.1 * MAX_SPEED)
    rightMotor.setVelocity(0.1 * MAX_SPEED)

    print("Connected to Webots simulation. Robot name:", robot.getName())

    # Main control loop
    while robot.step(TIME_STEP) != -1:
        # Add your control logic here.
        pass

if __name__ == '__main__':
    main()