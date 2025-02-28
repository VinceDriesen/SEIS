from controller import Motor, Robot, PositionSensor, DistanceSensor
import math


class TurtleBot:
    def __init__(self, robot: Robot, timeStep: int, maxSpeed: float):
        self.robot = robot
        self.timeStep = timeStep
        self.maxSpeed = maxSpeed
        self.leftMotor: Motor = self.robot.getDevice("left wheel motor")
        self.rightMotor: Motor = self.robot.getDevice("right wheel motor")
        self.radius = 0.033
        self.distanceBetweenWheels = 0.1775
        self._enableSensors()

        self.rightMotor.setPosition(float("inf"))
        self.leftMotor.setPosition(float("inf"))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.robot.step(self.timeStep)

        # X-waarde, Y-waarde, Hoek-waarde
        self.position = [0, 0, 0]

        # Parameter om max velocity mee te vermenigvuldigen
        self.velocityNorm = 0.3

    def _enableSensors(self):
        """This is a help function, do not touch it. Thank you
        This funciton initializes all the sensor to be used
        """
        self.frontDistSens: DistanceSensor = self.robot.getDevice(
            "front distance sensor"
        )
        self.rearDistSens: DistanceSensor = self.robot.getDevice("rear distance sensor")
        self.leftDistSens: DistanceSensor = self.robot.getDevice("left distance sensor")
        self.rightDistSens: DistanceSensor = self.robot.getDevice(
            "right distance sensor"
        )
        self.leftMotorSens: PositionSensor = self.robot.getDevice("left wheel sensor")
        self.rightMotorSens: PositionSensor = self.robot.getDevice("right wheel sensor")

        self.frontDistSens.enable(self.timeStep)
        self.rearDistSens.enable(self.timeStep)
        self.leftDistSens.enable(self.timeStep)
        self.rightDistSens.enable(self.timeStep)
        self.leftMotorSens.enable(self.timeStep)
        self.rightMotorSens.enable(self.timeStep)

    def getPosition(self) -> dict[str, float]:
        return {
            "x_value": self.position[0],
            "y_value": self.position[1],
            "theta_value": self.position[2],
        }

    def _rotate(self, angle: float):
        """This is a help function, do not touch it. Thank you
        This function rotates the robots in degrees. Use the movePosition function to move the robot

        Args:
            angle (float): Angle to rotate in degrees
        """
        # Idk als dit een goeie fix is, maar werkt nu wel op het moment
        if -0.01 < angle < 0.01:
            return
        angleRadians = math.radians(angle)
        angleVelocity = self.velocityNorm * self.maxSpeed

        startLeftEncoder = self.leftMotorSens.getValue()
        startRightEncoder = self.rightMotorSens.getValue()

        print(startLeftEncoder, startRightEncoder)

        targetRotation = angleRadians

        self.leftMotor.setVelocity(angleVelocity)
        self.rightMotor.setVelocity(-angleVelocity)

        while self.robot.step(self.timeStep) != -1:
            currentLeftEncoder = self.leftMotorSens.getValue()
            currentRightEncoder = self.rightMotorSens.getValue()

            leftRotationChange = currentLeftEncoder - startLeftEncoder
            rightRotationChange = currentRightEncoder - startRightEncoder

            currentRotation = (
                (rightRotationChange - leftRotationChange)
                * self.radius
                / self.distanceBetweenWheels
            )
            print(currentRotation, targetRotation)
            if abs(currentRotation) >= abs(targetRotation):
                break

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

    def _move(self, distance: float):
        """This is a help function, do not touch it. Thank you
        This function moves the robots in meters. Use the movePosition function to move the robot

        Args:
            distance (float): Distance in meters
        """
        linearVelocity = self.velocityNorm * self.maxSpeed

        startLeftEncoder = self.leftMotorSens.getValue()
        startRightEncoder = self.rightMotorSens.getValue()

        print(startLeftEncoder, startRightEncoder)

        targetDistance = distance

        self.leftMotor.setVelocity(linearVelocity)
        self.rightMotor.setVelocity(linearVelocity)

        while self.robot.step(self.timeStep) != -1:
            currentLeftEncoder = self.leftMotorSens.getValue()
            currentRightEncoder = self.rightMotorSens.getValue()

            leftDistanceTravelled = (
                currentLeftEncoder - startLeftEncoder
            ) * self.radius
            rightDistanceTravelled = (
                currentRightEncoder - startRightEncoder
            ) * self.radius

            currentDistance = (leftDistanceTravelled + rightDistanceTravelled) / 2.0
            print(currentDistance, targetDistance)
            if abs(currentDistance) >= abs(targetDistance):
                break

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

    def movePosition(self, x: float, y: float, angle: float):
        """
        This is a relative move function, from the current position, move x meters, y meters and or a new angle position.
        These can be None, if you only want to move in the x direction. De angle operator wordt EERST uitgevoerd, NIET ER NA!!
        Args:
            x (float): relative x movement
            y (float): relative y movement
            angle (float): relative angle movement
        """

        target_x = self.position[0] + (x if x is not None else 0)
        target_y = self.position[1] + (y if y is not None else 0)
        target_angle = (
            self.position[2] + math.radians(angle)
            if angle is not None
            else self.position[2]
        )

        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        target_distance = math.sqrt(dx**2 + dy**2)
        target_rotation = math.atan2(dy, dx) - self.position[2]

        target_rotation = self.normalizeAngle(target_rotation)

        if angle is not None:
            self._rotate(angle)

        if x is not None or y is not None:
            self._rotate(math.degrees(target_rotation))
            self._move(target_distance)

        self.position[0] = target_x
        self.position[1] = target_y
        self.position[2] = target_angle

    def normalizeAngle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
