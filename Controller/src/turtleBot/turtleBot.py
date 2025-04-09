import threading
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from matplotlib import pyplot as plt
import numpy as np
from controller import Motor, Robot, PositionSensor, DistanceSensor, Lidar, Compass, GPS
import math
from .lidar import LidarFunctions


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

        # # Parameters Occupany Map
        # self.map_size = 6 # Physical Map Size
        # self.map_resolution = 50 # Cells per meter
        # self.grid_size = int(self.map_size * self.map_resolution)
        # self.map_offset = self.grid_size // 2
        # self.map_lock = threading.Lock()
        
        # self.occupancy_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        x_gps, y_gps, _ = self.get_gps_position()  # Ignore Z coordinate
        theta = self.get_heading_from_compass()  # Updated heading
        self.position = list((x_gps, y_gps, theta))
        self.lidarFunc = LidarFunctions()

        # Parameter om max velocity mee te vermenigvuldigen
        self.velocityNorm = 0.3
        
        
        self.lidarFunc.scan(self.lidarSens, self.get_position())


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
        
        self.compass: Compass = self.robot.getDevice("compass")
        self.gps: GPS = self.robot.getDevice("gps")

        self.frontDistSens.enable(self.timeStep)
        self.rearDistSens.enable(self.timeStep)
        self.leftDistSens.enable(self.timeStep)
        self.rightDistSens.enable(self.timeStep)
        self.leftMotorSens.enable(self.timeStep)
        self.rightMotorSens.enable(self.timeStep)
        self.lidarSens.enable(self.timeStep)
        self.compass.enable(self.timeStep)
        self.gps.enable(self.timeStep)

    
    def get_heading_from_compass(self):
        x, y, _ = self.compass.getValues()
        # Laat deze 1/2pi staan, dit is aangezien compass zijn noorde legt tov de y-as en de map zijn noorde tov de x-as is
        heading = math.atan2(y, x) - 1/2 * math.pi
        return self.normalizeAngle(heading)
    
    def start_lidar(self):
        self.lidarFunc.scan(self.lidarSens, self.get_position())


    def get_gps_position(self):
        return self.gps.getValues()


    def get_position(self) -> dict[str, float]:
        return {
            "x_value": self.position[0],
            "y_value": self.position[1],
            "theta_value": self.position[2] - math.pi,
        }


    def _rotate(self, angle: float):
        """This is a help function, do not touch it. Thank you
        This function rotates the robot counter-clockwise (CCW) for positive angles
        and clockwise (CW) for negative angles. Use the movePosition function to move the robot.

        Args:
            angle (float): Angle to rotate in degrees
        """
        # If the angle is too small, skip rotation
        if -0.01 < angle < 0.01:
            return

        angleRadians = math.radians(angle)
        angleVelocity = self.velocityNorm * self.maxSpeed

        startLeftEncoder = self.leftMotorSens.getValue()
        startRightEncoder = self.rightMotorSens.getValue()

        targetRotation = angleRadians

        # Positive angle -> CCW rotation, Negative angle -> CW rotation
        if angle > 0:
            self.leftMotor.setVelocity(-angleVelocity)
            self.rightMotor.setVelocity(angleVelocity)
        else:
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

    def move_to_position(self, x: float, y: float):
        """
        Move robot to target position (x,y) where:
        - 0 = occupied
        - 1 = free
        """
        # Get current position
        current_pos = self.get_position()
        print(f"\n=== MOVING TO ({x:.2f}, {y:.2f}) FROM ({current_pos['x_value']:.2f}, {current_pos['y_value']:.2f}) ===")

        # Get occupancy grid (0=occupied, 1=free)
        grid_prob, extent = self.lidarFunc.get_occupancy_grid()
        
        # Convert to binary grid (INVERTED since 0=occupied)
        binary_grid = 1 - (grid_prob > 0.8).astype(np.int8)
        print(f"Grid stats: Size={binary_grid.shape}, Free%={np.mean(binary_grid)*100:.1f}%")

        # Coordinate conversion
        def real_to_grid(real_x, real_y):
            grid_x = int((real_x - extent[0]) / (extent[1]-extent[0]) * (binary_grid.shape[1]-1))
            grid_y = int((real_y - extent[2]) / (extent[3]-extent[2]) * (binary_grid.shape[0]-1))
            return np.clip(grid_x, 0, binary_grid.shape[1]-1), np.clip(grid_y, 0, binary_grid.shape[0]-1)

        # Convert positions
        start_x, start_y = real_to_grid(current_pos['x_value'], current_pos['y_value'])
        end_x, end_y = real_to_grid(x, y)
        print(f"Grid coords: Start=({start_x},{start_y}), End=({end_x},{end_y})")
        print(f"Cell values: Start={binary_grid[start_y, start_x]}, End={binary_grid[end_y, end_x]}")

        # Check if target is valid
        if binary_grid[end_y, end_x] == 0:
            print(f"ERROR: Target cell ({end_x},{end_y}) is occupied!")
            return False

        # Create pathfinding grid (transpose for correct x,y)
        grid = Grid(matrix=binary_grid.T)
        start = grid.node(start_x, start_y)
        end = grid.node(end_x, end_y)

        # Find path
        finder = AStarFinder()
        path, _ = finder.find_path(start, end, grid)
        print(path)
        
        if not path:
            print("ERROR: No path found! Showing area around target:")
            print(binary_grid[max(0,end_y-3):end_y+3, max(0,end_x-3):end_x+3])
            return False

        # Execute path
        for i, (grid_x, grid_y) in enumerate(path):
            target_x = (grid_x) / (binary_grid.shape[1]-1)+ (extent[0] / (extent[1]-extent[0]))
            target_y = (grid_y) / (binary_grid.shape[0]-1)+ (extent[2] / (extent[3]-extent[2]))
            # target_x = extent[0] + (grid_x / (binary_grid.shape[1]-1)) * (extent[1]-extent[0])
            # target_y = extent[2] + (grid_y / (binary_grid.shape[0]-1)) * (extent[3]-extent[2])
            
            dx = target_x - current_pos['y_value']
            dy = target_y - current_pos['x_value']
            
            if math.hypot(dx, dy) > 0.05:  # Skip tiny movements
                self.move_position(dx, dy, None)
                current_pos = self.get_position()

        print("Reached target position!")
        return True
        
    def move_position(self, x: float, y: float, angle: float):
        """
        This is a relative move function, from the current position, move x meters, y meters and or a new angle position.
        For reference, the X and Y coordinates are from the absolute X and Y of the map. This is X-axis in Red and Y-axis in Green
        The rotation is with math convention, so counter clock wise is positive and clock wise is negative.
        These can be None, if you only want to move in the x direction. The Angle operator will be run first, then the x and y!
        Args:
            x (float): relative x movement
            y (float): relative y movement
            angle (float): relative angle movement
        """

        target_x = self.position[0] + (x if x is not None else 0)
        target_y = self.position[1] + (y if y is not None else 0)

        if angle is not None and angle != 0:
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
        
        x_gps, y_gps, _ = self.get_gps_position()  # Ignore Z coordinate
        self.position[0] = x_gps
        self.position[1] = y_gps  # Now using Y for vertical in 2D map
        self.position[2] = self.get_heading_from_compass()  # Updated heading
        
        print(f"Position: {self.get_position()}")
        self.lidarFunc.scan(self.lidarSens, self.get_position())


    def normalizeAngle(self, angle):
        return angle % (2 * math.pi)