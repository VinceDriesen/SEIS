import math
from controller import Lidar
import numpy as np
import matplotlib.pyplot as plt
import threading

def bresenham(start, end):
    """Bresenham's line algorithm voor vrije ruimte updates"""
    x0, y0 = start
    x1, y1 = end
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    err = dx - dy
    
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points

class OccupancyGrid:
    def __init__(self):
        self.map_size = 6 # meters
        self.map_resolution = 0.03  # meters per cell
        self.grid_cells = int(self.map_size / self.map_resolution)
        self.grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.float32)
        self.free_prob = 0.4
        self.occ_prob = 0.7

    def world_to_grid(self, world_coords, position):
        """Zet wereldcoördinaten om naar grid-coördinaten"""
        grid_x = int(((world_coords[0] + self.map_size/2) / self.map_resolution))
        grid_y = int(((world_coords[1] + self.map_size/2) / self.map_resolution))
        return np.clip([grid_x, grid_y], 0, self.grid_cells-1).astype(int)

    def update_grid(self, sensor_pos, hits):
        """Werk de occupancy grid bij op basis van Lidar-metingen"""
        for hit in hits:
            line = bresenham(sensor_pos, hit)
            for cell in line[:-1]:  # Alle cellen behalve de laatste
                self.update_cell(cell, self.free_prob)
            if line:  # Laatste cel als occupied markeren
                self.update_cell(line[-1], self.occ_prob)

    def update_cell(self, cell, probability):
        log_odds = np.log(probability / (1 - probability))
        self.grid[cell[0], cell[1]] += log_odds
        self.grid = np.clip(self.grid, -10, 10)

class LidarFunctions:
    def __init__(self):
        self.occupancyGrid = OccupancyGrid()
        self.visualization_thread = threading.Thread(target=self.visualize_grid, daemon=True)
        self.visualization_thread.start()

    def get_lidar_global_coord_values(self, lidarSensor: Lidar, position):
        """Verwerkt de Lidar-metingen en zet ze om naar globale coördinaten"""
        scan = lidarSensor.getRangeImage()
        fov = lidarSensor.getFov()
        angle_increment = fov / (lidarSensor.getHorizontalResolution() - 1)
        
        coords = []
        for i in range(lidarSensor.getHorizontalResolution()):
            angle = -fov/2 + angle_increment * i
            distance = scan[i]
            if distance > 0:
                coords.append(self.local_to_global(distance, (angle + position['theta_value']), position))
        return coords

    def local_to_global(self, distance, lidar_angle, position):
        """
        Zet lokale LiDAR-coördinaten om naar wereldcoördinaten.

        :param distance: De afstand gemeten door de LiDAR.
        :param lidar_angle: De hoek van de LiDAR-meting (in radialen).
        :param position: De positie en oriëntatie van de robot [x, y, theta].
        :return: Globale coördinaten [x, y].
        """
        # Lokale coördinaten berekenen (in het robotframe)
        x_local = - distance * math.sin(lidar_angle)
        y_local = distance * math.cos(lidar_angle)

        x_global = x_local + position['x_value']
        y_global = y_local + position['y_value']
        return [x_global, y_global]

    def get_robot_position_grid(self, position):
        """Berekent de sensorpositie in de occupancy grid op basis van de absolute wereldcoördinaten"""
        return self.occupancyGrid.world_to_grid([position['x_value'], position['y_value']], position)

    def scan(self, lidarSensor: Lidar, position):
        """Voert een scan uit en werkt de occupancy grid bij"""
        # Transformeer lidar-metingen naar globale coördinaten
        sensor_pos = self.get_robot_position_grid(position)
        global_coords = self.get_lidar_global_coord_values(lidarSensor, position)
        
        # Update grid met gescande punten
        hits = [self.occupancyGrid.world_to_grid(c, position) for c in global_coords]
        self.occupancyGrid.update_grid(sensor_pos, hits)

    def visualize_grid(self):
        plt.figure()
        while True:
            plt.clf()
            grid_prob = 1 - 1/(1 + np.exp(self.occupancyGrid.grid))  # Log-odds naar probability
            extent = [-self.occupancyGrid.map_size/2, self.occupancyGrid.map_size/2,
                      -self.occupancyGrid.map_size/2, self.occupancyGrid.map_size/2]
            plt.imshow(grid_prob.T, extent=extent, origin='lower', cmap='binary', vmin=0, vmax=1)
            plt.colorbar(label='Occupancy Probability')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('Occupancy Grid')
            plt.draw()
            plt.pause(0.1)