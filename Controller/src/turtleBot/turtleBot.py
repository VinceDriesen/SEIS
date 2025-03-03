from matplotlib import pyplot as plt
import numpy as np
from controller import Motor, Robot, PositionSensor, DistanceSensor, Lidar
import math

class TurtleBot:
    """
    A class representing a TurtleBot robot in a Webots simulation.

    The TurtleBot is equipped with two wheel motors and multiple distance sensors.
    It can move, sense its environment, and update its position accordingly. 
    """
    
    def __init__(self, robot: Robot, timeStep: int, maxSpeed: float):
        """Initializes The TurtleBot

        Args:
            robot (Robot): Webots Turtlebot instance
            timeStep (int): Simulation Timestep
            maxSpeed (float): Maximun wheel speed 
        """
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
        
        # Define occupancy map parameters.
        self.map_size = 4                                           # Size of map in meters abs(-x, +x)
        self.map_resolution = 20                                    # Amount of div per meter
        self.grid_size = self.map_size * self.map_resolution
        self.map_offset = self.grid_size // 2                       # Center index corresponding to (0,0)
        self.occurancy_map = np.zeros((self.grid_size, self.grid_size))

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
        
        self.lidarSens: Lidar = self.robot.getDevice("LDS-01")
        self.lidarMotor1: Motor = self.robot.getDevice("LDS-01_main_motor")
        self.lidarMotor2: Motor = self.robot.getDevice("LDS-01_secondary_motor")

        self.frontDistSens.enable(self.timeStep)
        self.rearDistSens.enable(self.timeStep)
        self.leftDistSens.enable(self.timeStep)
        self.rightDistSens.enable(self.timeStep)
        self.leftMotorSens.enable(self.timeStep)
        self.rightMotorSens.enable(self.timeStep)
        self.lidarSens.enable(self.timeStep)

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

        targetDistance = distance
        self.leftMotor.setVelocity(linearVelocity)
        self.rightMotor.setVelocity(linearVelocity)
        
        prevDistance = 0.0

        while self.robot.step(self.timeStep) != -1:
            currentLeftEncoder = self.leftMotorSens.getValue()
            currentRightEncoder = self.rightMotorSens.getValue()
            frontSensorValue = self.frontDistSens.getValue()
            
            if frontSensorValue > 1000 - (self.distanceBetweenWheels * 1.1 * 1000):
                print("Broken Off Movement!")
                break
            
            leftDistanceTravelled = (
                currentLeftEncoder - startLeftEncoder
            ) * self.radius
            rightDistanceTravelled = (
                currentRightEncoder - startRightEncoder
            ) * self.radius
            currentDistance = (leftDistanceTravelled + rightDistanceTravelled) / 2.0

            delta = currentDistance - prevDistance

            self.position[0] += delta * math.cos(self.position[2])
            self.position[1] += delta * math.sin(self.position[2])
            
            prevDistance = currentDistance
            
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
        
        if angle is not None:
            desired_heading = self.normalizeAngle(self.position[2] + math.radians(angle))
        else:
            desired_heading = math.atan2(target_y - self.position[1], target_x - self.position[0])
        
        rotation_needed = math.degrees(self.normalizeAngle(desired_heading - self.position[2]))
        
        self._rotate(rotation_needed)
        
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)

        self._move(distance)

        # self.position[0] = target_x
        # self.position[1] = target_y
        self.position[2] = desired_heading

    def normalizeAngle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
        
    def __get_lidar_scan(self) -> np.ndarray:
        """
        Transform raw LIDAR data so that it is correctly sorted with angles 0 to 359 degrees (with 0° at the top/north).

        Returns:
            np.ndarray: The transformed LIDAR data array.
        """
        lidar_array = np.array(self.lidarSens.getRangeImage(), dtype=float)
        
        # Replace infinite values with minRange
        lidar_array[lidar_array == np.inf] = np.nan
        
        # Reverse the order of the data (flip the scan)
        transformed = lidar_array[::-1]
        
        return transformed
    
    
    def _add_to_occurancy_map(self, x_value: float, y_value: float, cell_value: float = 1) -> None:
        """Adds the cell_value to the occurancy map at the specified position

        Args:
            x_value (float): x positions in m
            y_value (float): y posittion in m
            cell_value (float, optional): propability of cell having an object, default = 1 
        """
        i = int(x_value * self.map_resolution) + self.map_offset  # Column index
        j = int(y_value * self.map_resolution) + self.map_offset     # Row index
        if 0 <= i < self.grid_size and 0 <= j < self.grid_size:
            self.occurancy_map[j, i] = cell_value
        else:
            print(f'Warning: ({x_value}, {y_value}) is out of map bounds.')
        
    def _scan_lidar_event(self):
        lidar_data = self.__get_lidar_scan()
        angles = np.linspace(0, 2 * np.pi, len(lidar_data), endpoint=False)
        
        robot_pos_x = self.position[0]
        robot_pos_y = self.position[1]
        robot_pos_theta = self.position[2]
        
        for dist, angle in zip(lidar_data, angles):
            if np.isnan(dist):
                continue
            x_dest = robot_pos_x + dist * math.cos(robot_pos_theta + angle)
            y_dest = robot_pos_y + dist * math.sin(robot_pos_theta + angle)
            self._add_to_occurancy_map(x_dest, y_dest)
            
    def display_occupancy_map(self):
        """
        Display the occupancy map in real time using Matplotlib's interactive mode.
        The origin of the display is centered in physical units (meters).
        """
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()

        # Set extent in physical units (meters): from -map_size/2 to map_size/2.
        half_map_meters = self.map_size / 2  # e.g., 4/2 = 2
        extent = [-half_map_meters, half_map_meters, -half_map_meters, half_map_meters]

        # Display the occupancy map using imshow.
        img = ax.imshow(self.occurancy_map, cmap='gray', origin='lower', 
                        interpolation='nearest', extent=extent)

        plt.title("Occupancy Map")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.colorbar(img)
        plt.show()

        while True:
            # Update the image data with the current occupancy map.
            img.set_data(self.occurancy_map)
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.01)
