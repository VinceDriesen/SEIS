import threading
from typing import List, Tuple
from matplotlib import pyplot as plt
import numpy as np
from controller import Motor, Robot, PositionSensor, DistanceSensor, Lidar, Compass, GPS
import math

class TurtleBot:
    """
    A class representing a TurtleBot robot in a Webots simulation.

    The TurtleBot is equipped with two wheel motors and multiple distance sensors.
    It can move, sense its environment, and update its position accordingly.
    """

    def __init__(self, robot: Robot, timeStep: int, maxSpeed: float, initial_position: tuple[float, float, float] = (0, 0, 0)):
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

        # Parameters Occupany Map
        self.map_size = 6 # Physical Map Size
        self.map_resolution = 50 # Cells per meter
        self.grid_size = int(self.map_size * self.map_resolution)
        
        self.occupancy_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.obstacle_treshold = 2

        self.position = list(initial_position)

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
        # self.lidarMotor1: Motor = self.robot.getDevice("LDS-01_main_motor")
        # self.lidarMotor2: Motor = self.robot.getDevice("LDS-01_secondary_motor")
        
        self.compass: Compass = self.robot.getDevice("compass")
        self.gps: GPS = self.robot.getDevice("gps")

        self.frontDistSens.enable(self.timeStep)
        self.rearDistSens.enable(self.timeStep)
        self.leftDistSens.enable(self.timeStep)
        self.rightDistSens.enable(self.timeStep)
        self.leftMotorSens.enable(self.timeStep)
        self.rightMotorSens.enable(self.timeStep)
        self.compass.enable(self.timeStep)
        self.gps.enable(self.timeStep)

        self.lidarSens.enable(self.timeStep)
        # self.lidarSens.enablePointCloud()
    
    def get_heading_from_compass(self):
        x, y, _ = self.compass.getValues()
        heading = math.atan2(y, x)
        return self.normalizeAngle(heading)


    def get_gps_position(self):
        return self.gps.getValues()


    def get_position(self) -> dict[str, float]:
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

            leftDistanceTravelled = (currentLeftEncoder - startLeftEncoder) * self.radius
            rightDistanceTravelled = (currentRightEncoder - startRightEncoder) * self.radius
            currentDistance = (leftDistanceTravelled + rightDistanceTravelled) / 2.0

            delta = currentDistance - prevDistance

            # Update position using wheel odometry
            self.position[0] += delta * math.cos(self.position[2])
            self.position[1] += delta * math.sin(self.position[2])

            prevDistance = currentDistance

            if abs(currentDistance) >= abs(targetDistance):
                break

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)


    
    def move_position(self, x: float, y: float, angle: float):
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
            desired_heading = self.normalizeAngle(
                self.position[2] + math.radians(angle)
            )
        else:
            desired_heading = math.atan2(
                target_y - self.position[1], target_x - self.position[0]
            )

        rotation_needed = math.degrees(
            self.normalizeAngle(desired_heading - self.position[2])
        )

        self._rotate(rotation_needed)

        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)

        self._move(distance)
        
        x_gps, y_gps, _ = self.get_gps_position()
        self.position[0] = x_gps
        self.position[1] = y_gps
        self.position[2] = self.get_heading_from_compass()    
        self.scan_lidar()
        self.visualize_occupancy_map()


    def normalizeAngle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    
    def coords_to_grid(self, coords: Tuple[float, float]):
        grid_x = int((coords[0] + self.map_size / 2) * self.map_resolution)
        grid_y = int((coords[1] + self.map_size / 2) * self.map_resolution)
        print(grid_x, grid_y)
        return grid_x, grid_y
    
    def scan_lidar(self):
        ranges = self.lidarSens.getRangeImage()
        
        fov = self.lidarSens.getFov()
        angle_increment = fov / (self.lidarSens.getHorizontalResolution() - 1)
        
        # Create a temporary array to mark free space between robot and obstacles
        temp_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # Get robot position in grid coordinates
        robot_grid_x, robot_grid_y = self.coords_to_grid((self.position[0], self.position[1]))
        
        for i in range(self.lidarSens.getHorizontalResolution()):
            # Calculate angle in the global reference frame
            scan_angle = self.position[2] + (-fov / 2 + angle_increment * i)
            distance = ranges[i]
            
            if distance > 0 and distance < 10:  # Ignore readings that are too far
                # Calculate endpoint in global coordinates directly
                global_x = self.position[0] + distance * math.cos(scan_angle)
                global_y = self.position[1] + distance * math.sin(scan_angle)
                
                # Convert to grid coordinates
                grid_x, grid_y = self.coords_to_grid((global_x, global_y))
                
                # Check if coordinates are within map bounds
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    # Mark obstacle cells
                    if distance < self.obstacle_treshold:
                        temp_map[grid_y, grid_x] = 1
                    
                    # Use Bresenham's line algorithm to mark free space between robot and obstacle
                    self.draw_line(robot_grid_x, robot_grid_y, grid_x, grid_y, temp_map)
        
        # Update the main occupancy map with the new observations
        # We only update cells that are marked in temp_map
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if temp_map[y, x] == 1:
                    self.occupancy_map[y, x] = 1  # Obstacle
                elif temp_map[y, x] == 2:
                    self.occupancy_map[y, x] = 0  # Free space

    def draw_line(self, x0, y0, x1, y1, temp_map):
        """
        Bresenham's line algorithm to mark free space between points
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            # Mark all points on the line except the endpoint as free space
            if (x0 != x1 or y0 != y1) and 0 <= x0 < self.grid_size and 0 <= y0 < self.grid_size:
                if temp_map[y0, x0] == 0:  # Only mark as free if not already marked as obstacle
                    temp_map[y0, x0] = 2  # Mark as free space
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy  
                                  
    def visualize_occupancy_map(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.occupancy_map, cmap='binary', origin='lower')
        
        # Mark robot position
        robot_grid_x, robot_grid_y = self.coords_to_grid((self.position[0], self.position[1]))
        plt.plot(robot_grid_x, robot_grid_y, 'ro', markersize=10)
        
        plt.title('Occupancy Map')
        plt.xlabel('X Grid Coordinate')
        plt.ylabel('Y Grid Coordinate')
        plt.savefig('occupancy_map.png')  # Save to file for Webots
        plt.close()