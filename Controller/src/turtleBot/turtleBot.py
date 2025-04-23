import threading
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from matplotlib import pyplot as plt
import numpy as np
from controller import Motor, Robot, PositionSensor, DistanceSensor, Lidar, Compass, GPS
import math
from .lidar import LidarFunctions


from collections import deque


class TurtleBot:
    """
    A class representing a TurtleBot robot in a Webots simulation.

    The TurtleBot is equipped with two wheel motors and multiple distance sensors.
    It can move, sense its environment, and update its position accordingly.
    """

    def __init__(self, name, robot: Robot, timeStep: int, maxSpeed: float):
        """Initializes The TurtleBot

        Args:
            robot (Robot): Webots Turtlebot instance
            timeStep (int): Simulation Timestep
            maxSpeed (float): Maximun wheel speed
        """
        self.name = name
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
        self.nonMeasuredPosition = [0,0,0]
        
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
        heading = math.atan2(x,y)
        # return heading
        return self.normalizeAngle(heading)
    
    def start_lidar(self):
        self.lidarFunc.scan(self.lidarSens, self.get_position())


    def get_gps_position(self):
        return self.gps.getValues()


    def get_position(self) -> dict[str, float]:
        return {
            "x_value": self.position[0],
            "y_value": self.position[1],
            "theta_value": self.position[2],
        }

    def execute_task(self, task: dict):
        print(f"Executing task in robot: {task}")
        if task["type"] == "move_to":
            x = task["parameters"]["x"]
            y = task["parameters"]["y"]
            self.move_to_position(x, y)
        elif task["type"] == "turn":
            angle = task["parameters"]["angle"]
            self.move_position(0, 0, angle)
        elif task["type"] == "explore_environment":
            self.explore_environment()   
        else:
            raise ValueError(f"Unknown task type: {task['type']}")


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

            #Blijf van die 0.02 af, de robot schoot gwn iets te ver door, dus die 0.02 is gdn adv tests
            if (abs(currentRotation) + 0.02) >= abs(targetRotation):
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

            # if frontSensorValue > 300 - (self.distanceBetweenWheels * 1.1 * 1000):
            #     print("Broken Off Movement!")
            #     break

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
        
        
    def fix_position(self):
        """
        Fix the position of the robot in the map.
        This function is used to correct the position of the robot in the occupancy map.
        """
        
        
        x_gps, y_gps, _ = self.get_gps_position()
        
        # Calculate the difference between the current position and the target position
        dx = self.nonMeasuredPosition[0] - x_gps
        
        
        if abs(dx) > 0.05:
            print(f"correcting dx: {dx}")
            self.move_position(dx, 0, 0, safePosition=False)
        x_gps, y_gps, _ = self.get_gps_position()
        dy = self.nonMeasuredPosition[1] - y_gps
        if abs(dy) > 0.05:
            print(f"correcting dy: {dy}")
            self.move_position(0, dy, 0, safePosition=False)
        rotation = self.get_heading_from_compass()
        d_rot = self.nonMeasuredPosition[2] - rotation
        if d_rot < -math.pi:
            d_rot += 2 * math.pi
        if d_rot > math.pi:
            d_rot -= 2 * math.pi
        if abs(d_rot) > 0.03:
            print(f"correcting d_rot: {d_rot}")
            self.move_position(0, 0, d_rot, safePosition=False)
        
        
    def simplify_path(self, path, grid):
        """
        Vereenvoudig het pad door punten te verwijderen waar een rechte lijn mogelijk is.
        Gebruikt Bresenham's algoritme om de lijn te checken op obstakels.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_index = 0

        while current_index < len(path) - 1:
            furthest_index = current_index + 1  # Minstens één stap vooruit
            # Zoek het verste punt vanaf current_index met vrij zicht
            for next_index in range(len(path)-1, current_index, -1):
                if self.has_line_of_sight(path[current_index], path[next_index], grid):
                    furthest_index = next_index
                    break
            simplified.append(path[furthest_index])
            current_index = furthest_index

        return simplified

    def has_line_of_sight(self, node_a, node_b, grid):
        """Controleer of er een rechte lijn is tussen twee nodes zonder obstakels."""
        x0, y0 = node_a.x, node_a.y
        x1, y1 = node_b.x, node_b.y
        line = self.bresenham_line(x0, y0, x1, y1)

        for (x, y) in line:
            # Check of de node binnen de grid grenzen valt
            if x < 0 or y < 0 or x >= grid.width or y >= grid.height:
                return False
            if not grid.node(x, y).walkable:
                return False
        return True

    def bresenham_line(self, x0, y0, x1, y1):
        """Genereer punten op de lijn tussen (x0,y0) en (x1,y1) met Bresenham's algoritme."""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # foutaccumulatie

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:  # horizontale stap
                if x0 == x1:
                    break
                err += dy
                x0 += sx
            if e2 <= dx:  # verticale stap
                if y0 == y1:
                    break
                err += dx
                y0 += sy
        return points
    


    def visualize_pathfinding_grid(self, grid, path=None, start=None, end=None):
        """
        Visualiseer de grid die gebruikt wordt door A* (pathfinding.core.grid.Grid).
        1 = walkable, 0 = wall. Het midden van de map wordt weergegeven als (0,0).
        """
        import numpy as np
        import matplotlib.pyplot as plt

        matrix = np.array([[1 if node.walkable else 0 for node in col] for col in grid.nodes])
        matrix = matrix  # Transpose terug naar originele oriëntatie

        if path:
            for node in path:
                x, y = node.x, node.y
                if 0 <= y < matrix.shape[0] and 0 <= x < matrix.shape[1]:
                    matrix[y, x] = 0.5  # Pad = grijs

        if start:
            matrix[start[1], start[0]] = 0.2  # Start = donkergrijs
        if end:
            matrix[end[1], end[0]] = 0.8      # End = lichtgrijs

        # Zet het midden van de map op (0,0)
        extent = [-matrix.shape[1] // 2, matrix.shape[1] // 2, -matrix.shape[0] // 2, matrix.shape[0] // 2]

        plt.figure(figsize=(8, 8))
        plt.title("A* Grid Visualization (used by pathfinder)")
        plt.imshow(matrix, cmap='gray', origin='lower', extent=extent)

        # Highlight the path in blue
        if path:
            path_x = [node.x - matrix.shape[1] // 2 for node in path]
            path_y = [node.y - matrix.shape[0] // 2 for node in path]
            plt.plot(path_x, path_y, color='blue', linewidth=2, label="Path")

        plt.xlabel("X (0,0 is center)")
        plt.ylabel("Y (0,0 is center)")
        plt.axhline(0, color='red', linewidth=0.5)  # X-as
        plt.axvline(0, color='green', linewidth=0.5)  # Y-as
        plt.legend()
        plt.show()

    def add_buffer_to_grid(self, grid, extent, buffer_distance=0.2):
        """Verbeterde versie met gegarandeerde minimale buffer"""
        cell_size = (extent[1] - extent[0]) / grid.shape[1]
        buffer_cells = max(1, int(round(buffer_distance / cell_size)))  # Minimaal 1 cel, met afronding
        
        # Maak een kernel voor circulaire buffer
        y, x = np.ogrid[-buffer_cells:buffer_cells+1, -buffer_cells:buffer_cells+1]
        mask = x*x + y*y <= buffer_cells*buffer_cells
        
        buffered_grid = grid.copy()
        wall_coords = np.argwhere(grid == 0)
        
        for y, x in wall_coords:
            # Bepaal bounds van de buffer
            y_min = max(0, y - buffer_cells)
            y_max = min(grid.shape[0], y + buffer_cells + 1)
            x_min = max(0, x - buffer_cells)
            x_max = min(grid.shape[1], x + buffer_cells + 1)
            
            # Pas de buffer toe
            buffered_grid[y_min:y_max, x_min:x_max] *= ~mask[
                buffer_cells - (y - y_min): buffer_cells + (y_max - y),
                buffer_cells - (x - x_min): buffer_cells + (x_max - x)
            ]
        
        return buffered_grid

    def move_to_position(self, x: float, y: float):
        """
        Move robot to target position !absolute! (x,y), this will be done with A* pathfinding.
        """
        # Get current position
        current_pos = self.get_position()
        print(f"\n=== MOVING TO ({x:.2f}, {y:.2f}) FROM ({current_pos['x_value']:.2f}, {current_pos['y_value']:.2f}) ===")

        # Get occupancy grid (0=occupied, 1=free)
        grid_prob, extent = self.lidarFunc.get_occupancy_grid()
        
        # Convert to binary grid (INVERTED since 0=occupied)
        binary_grid = 1 - (grid_prob > 0.8).astype(np.int8)
        print(f"Grid stats: Size={binary_grid.shape}, Free%={np.mean(binary_grid)*100:.1f}%")

        # Voeg buffer toe rond muren (om een minimale afstand te behouden)
        buffered_grid = self.add_buffer_to_grid(binary_grid, extent)
        # buffered_grid = binary_grid

        # Coordinate conversion
        def real_to_grid(real_x, real_y):
            grid_x = int((real_x - extent[0]) / (extent[1]-extent[0]) * (buffered_grid.shape[1]))
            grid_y = int((real_y - extent[2]) / (extent[3]-extent[2]) * (buffered_grid.shape[0]))
            return np.clip(grid_x, 0, buffered_grid.shape[1]), np.clip(grid_y, 0, buffered_grid.shape[0])

        # Convert positions
        start_x, start_y = real_to_grid(current_pos['x_value'], current_pos['y_value'])
        end_x, end_y = real_to_grid(x, y)
        print(f"Grid coords: Start=({start_x},{start_y}), End=({end_x},{end_y})")
        print(f"Cell values: Start={buffered_grid[start_y, start_x]}, End={buffered_grid[end_y, end_x]}")

        # Check if target is valid
        if buffered_grid[end_y, end_x] == 0:
            print(f"ERROR: Target cell ({end_x},{end_y}) is occupied or too close to a wall!")
            return False

        # Create pathfinding grid (transpose for correct x,y)
        grid = Grid(matrix=buffered_grid.tolist())
        start = grid.node(start_x, start_y)
        end = grid.node(end_x, end_y)

        # Find path
        finder = AStarFinder()
        path, _ = finder.find_path(start, end, grid)
        # self.visualize_pathfinding_grid(grid, path=path, start=(start_x, start_y), end=(end_x, end_y))
        path = self.simplify_path(path, grid)
        
        print(path)
        
        if not path:
            print("ERROR: No path found! Showing area around target:")
            print(buffered_grid[max(0,end_y-3):end_y+3, max(0,end_x-3):end_x+3])
            return False

        # Execute path
        for i, (grid_x, grid_y) in enumerate(path):
            if i == 0: continue # Skip first point (already at start)
            target_x = (grid_x) / (buffered_grid.shape[1]) * (extent[1] - extent[0]) + extent[0]
            target_y = (grid_y) / (buffered_grid.shape[0]) * (extent[3] - extent[2]) + extent[2]
            
            dx = target_x - current_pos['x_value']
            dy = target_y - current_pos['y_value'] #Laat deze plug staan, dat klopt
            
            self.move_position(dx, dy, 0)
            current_pos = self.get_position()

        print("Reached target position!")
        return True

    
    
        
    def move_position(self, x: float, y: float, angle: float, safePosition: bool = True):
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
            # self.normalizeAngle(desired_heading - self.position[2])
            desired_heading - self.position[2]
        )
        
        if safePosition:
            if angle is not None and angle != 0:
                self.nonMeasuredPosition = [self.nonMeasuredPosition[0] + x, self.nonMeasuredPosition[1] + y, self.normalizeAngle(self.nonMeasuredPosition[2] + math.radians(angle))]
            else:
                self.nonMeasuredPosition = [self.nonMeasuredPosition[0] + x, self.nonMeasuredPosition[1] + y, self.normalizeAngle(desired_heading)]
            print(f"Non-measured position: {self.nonMeasuredPosition}")
            
        if abs(rotation_needed) > 180:
            rotation_needed -= 360 if rotation_needed > 0 else -360
            

        self._rotate(rotation_needed)

        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)

        self._move(distance)
        
        if not safePosition and (angle is None or angle == 0):
            self._rotate(-rotation_needed)
        
        x_gps, y_gps, _ = self.get_gps_position()  # Ignore Z coordinate
        self.position[0] = x_gps
        self.position[1] = y_gps  # Now using Y for vertical in 2D map
        self.position[2] = self.get_heading_from_compass()  # Updated heading
        
        print(f"Position: {self.get_position()}")
        
        self.fix_position()


    def normalizeAngle(self, angle): 
        return angle % (2 * math.pi)
    
    def explore_environment(self, num_candidates=30, cost_alpha=1.0):
        """Verkent de omgeving met Mutual Information gebaseerde doel selectie."""
        """
            J. M. Valero, J. del Cerro, A. Jardón, C. R. Quintero and J. L. Sanz, 
            "Mutual information-based exploration on continuous occupancy maps," 
            2016 IEEE International Conference on Robotics and Automation (ICRA), 
            Stockholm, Sweden, 2016, pp. 2807-2812, 
            doi: 10.1109/ICRA.2016.7487635.
        """
        print("Starting Mutual Information based exploration...")
        processed_targets = set() 
        tries = 0

        while self.robot.step(self.timeStep) != -1:
            print("geraakt hier!")
            current_pos_dict = self.get_position()
            self.lidarFunc.scan(self.lidarSens, current_pos_dict)

            if tries > 10:
                print("Too many tries, stopping exploration. It is probably done!.")
                break

            grid_obj = self.lidarFunc.occupancyGrid
            current_theta = current_pos_dict['theta_value']

            frontier_cells = grid_obj.find_frontier_cells()

            if not frontier_cells:
                print("No more frontiers found. Exploration might be stuck or complete.")
                break

            if len(frontier_cells) > num_candidates:
                indices = np.random.choice(len(frontier_cells), num_candidates, replace=False)
                candidate_grid_cells = [frontier_cells[i] for i in indices]
            else:
                candidate_grid_cells = frontier_cells

            entropy_grid = grid_obj.get_entropy_grid() 

            print(f"Evaluating {len(candidate_grid_cells)} candidate frontier cells...")
            candidate_info = []

            for candidate_cell in candidate_grid_cells:
                candidate_pose_grid = candidate_cell
                candidate_theta = current_theta

                observed_cells_indices = grid_obj.simulate_scan_from_pose(candidate_pose_grid, candidate_theta)

                information_gain = 0
                for r, c in observed_cells_indices:
                     if 0 <= r < grid_obj.grid_cells and 0 <= c < grid_obj.grid_cells:
                           information_gain += entropy_grid[r, c]

                candidate_world_coords = grid_obj.grid_to_world(candidate_cell)
                dx = candidate_world_coords[0] - current_pos_dict['x_value']
                dy = candidate_world_coords[1] - current_pos_dict['y_value']
                cost = math.sqrt(dx**2 + dy**2)

                if cost < 1e-6:
                    utility = information_gain 
                else:
                    utility = information_gain / (cost ** cost_alpha)

                candidate_info.append({
                    'cell': candidate_cell,
                    'world': candidate_world_coords,
                    'gain': information_gain,
                    'cost': cost,
                    'utility': utility
                })

            if not candidate_info:
                print("Warning: No valid candidates found after evaluation.")
                continue
            
            candidate_info.sort(key=lambda x: x['utility'], reverse=True)

            selected_candidate = None
            for candidate in candidate_info:
                 target_key = (round(candidate['world'][0], 2), round(candidate['world'][1], 2))
                 if target_key not in processed_targets:
                     selected_candidate = candidate
                     processed_targets.add(target_key)
                     break 
            if selected_candidate is None:
                print("All high-utility frontiers have been tried recently. Picking the best overall.")
                if candidate_info:
                     selected_candidate = candidate_info[0]
                else:
                     print("ERROR: No candidates available at all.")
                     break


            if selected_candidate:
                target_world = selected_candidate['world']
                print(f"Selected Target: {target_world} (Cell: {selected_candidate['cell']}) "
                      f"Utility: {selected_candidate['utility']:.2f} (Gain: {selected_candidate['gain']:.2f}, Cost: {selected_candidate['cost']:.2f})")

                success = self.move_to_position(target_world[0], target_world[1])

                if not success:
                    tries += 1
                    print(f"Failed to reach target {target_world}. Adding to processed targets.")
                else:
                    tries = 0
            else:
                 print("Could not select a new target.")
                 self.robot.step(self.timeStep * 10)

        print("Exploration loop finished.")