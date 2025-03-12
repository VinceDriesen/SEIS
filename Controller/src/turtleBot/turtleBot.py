import threading
from matplotlib import pyplot as plt
import numpy as np
from controller import Motor, Robot, PositionSensor, DistanceSensor, Lidar
import math
from .lidar import calculate_odometry_correction, transform_lidar_scan
from typing import List

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
        
        self.prev_lidar_scan = np.array([])

        # Parameters Occupany Map
        self.map_size = 6 # Physical Map Size
        self.map_resolution = 50 # Cells per meter
        self.grid_size = int(self.map_size * self.map_resolution)
        self.map_offset = self.grid_size // 2
        self.map_lock = threading.Lock()
        
        self.occupancy_map = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # X-waarde, Y-waarde, Hoek-waarde
        self.prev_position = list(initial_position)
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
        self.prev_position = self.position.copy()
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

            # Update position using wheel odometry
            self.position[0] += delta * math.cos(self.position[2])
            self.position[1] += delta * math.sin(self.position[2])

            prevDistance = currentDistance

            if abs(currentDistance) >= abs(targetDistance):
                break

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        
        self.robot.step(self.timeStep)
        current_scan = np.array(self.lidarSens.getRangeImage(), dtype=float)
        
        if len(self.prev_lidar_scan) > 0:
            dx, dy, dtheta = calculate_odometry_correction(current_scan, self.prev_lidar_scan)
            
            # Convert to global coordinates
            theta_prev = self.prev_position[2]
            dx_global = dx * math.cos(theta_prev) - dy * math.sin(theta_prev)
            dy_global = dx * math.sin(theta_prev) + dy * math.cos(theta_prev)
            
            # Update position with LiDAR correction
            self.position[0] = self.prev_position[0] + dx_global
            self.position[1] = self.prev_position[1] + dy_global
            self.position[2] = self.normalizeAngle(self.position[2] + dtheta)

        points_map = transform_lidar_scan(
            current_scan,
            (self.position[0],
            self.position[1],),
            self.position[2]
        )
        self._update_occupancy_map(points_map)
        
        self.prev_lidar_scan = current_scan.copy()
        self.prev_position = self.position.copy()

        # Normalize angle
        self.position[2] = self.normalizeAngle(self.position[2])

    
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
        self.position[2] = desired_heading


    def normalizeAngle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def _update_occupancy_map(self, points: np.ndarray):
        """Update map with proper coordinate conversion"""
        with self.map_lock:
            for x, y in points:
                # Convert world coordinates to grid indices
                grid_x = int((x + self.map_size/2) * self.map_resolution)
                grid_y = int((y + self.map_size/2) * self.map_resolution)
                
                print(f"World: ({x:.2f}, {y:.2f}) → Grid: ({grid_x}, {grid_y})")
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    self.occupancy_map[grid_x, grid_y] = 1
                
    def display_occupancy_map(self):
        """Visualization with proper updates and thread safety"""
        import matplotlib.pyplot as plt
        plt.ion()
        
        fig, ax = plt.subplots()
        extent = [-self.map_size/2, self.map_size/2, 
                -self.map_size/2, self.map_size/2]
        
        # Create initial plot with correct normalization
        img = ax.imshow(self.occupancy_map.T,
                    cmap='binary',
                    origin='lower',
                    extent=extent,
                    interpolation='none',
                    vmin=0,
                    vmax=1)
        
        ax.set_title("Live Occupancy Map")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        
        # Store references for clean updates
        fig.canvas.draw()
        background = fig.canvas.copy_from_bbox(fig.bbox)
        
        try:
            while True:
                # Update data with thread-safe access
                with self.map_lock:
                    updated_data = self.occupancy_map.T.copy()
                
                # Efficiently update visualization
                img.set_data(updated_data)
                
                # Restore background and redraw
                fig.canvas.restore_region(background)
                ax.draw_artist(img)
                fig.canvas.blit(fig.bbox)
                
                plt.pause(0.05)  # Maintain UI responsiveness
                
        except KeyboardInterrupt:
            plt.close('all')