import math
from controller import Lidar
import numpy as np
import matplotlib.pyplot as plt
import time
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
        self.map_size = 10  # meters
        self.map_resolution = 0.05  # meters per cell
        self.grid_cells = int(self.map_size / self.map_resolution)
        self.grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.float32)
        self.free_prob = 0.4
        self.occ_prob = 0.7

    def coords_to_grid(self, coords):
        grid_x = int((coords[0] + self.map_size/2) / self.map_resolution)
        grid_y = int((coords[1] + self.map_size/2) / self.map_resolution)
        return np.clip([grid_x, grid_y], 0, self.grid_cells-1).astype(int)

    def update_grid(self, sensor_pos, hits):
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
        scan = lidarSensor.getRangeImage()
        fov = lidarSensor.getFov()
        angle_increment = fov / (lidarSensor.getHorizontalResolution() - 1)
        
        coords = []
        for i in range(lidarSensor.getHorizontalResolution()):
            angle = -fov/2 + angle_increment * i
            distance = scan[i]
            if distance > 0:
                x_local = distance * math.cos(angle)
                y_local = distance * math.sin(angle)
                coords.append(self.local_to_global((x_local, y_local), position))
        return coords
    
    def local_to_global(self, local, position):
        x_global = position['x_value'] + local[0] * math.cos(position['theta_value']) - local[1] * math.sin(position['theta_value'])
        y_global = position['y_value'] + local[0] * math.sin(position['theta_value']) + local[1] * math.cos(position['theta_value'])
        return [x_global, y_global]

    def get_sensor_position(self, position):
        return self.occupancyGrid.coords_to_grid((position['x_value'], position['y_value']))

    def scan(self, lidarSensor: Lidar, position):
        # Robotpositie en oriëntatie
        x_robot = position['x_value']
        y_robot = position['y_value']
        theta_robot = position['theta_value']  # Converteer graden naar radialen indien nodig

        # Transformeer lidar-metingen
        global_coords = self.get_lidar_global_coord_values(lidarSensor, position)
        
        # Update grid
        sensor_pos = self.get_sensor_position(position)
        hits = [self.occupancyGrid.coords_to_grid(c) for c in global_coords]
        self.occupancyGrid.update_grid(sensor_pos, hits)

    def visualize_grid(self):
        while True:
            plt.clf()
            grid_prob = 1 - 1/(1 + np.exp(self.occupancyGrid.grid))  # Log-odds naar probability
            plt.imshow(grid_prob.T, origin='lower', cmap='binary', vmin=0, vmax=1)
            plt.colorbar(label='Occupancy Probability')
            plt.draw()
            plt.pause(0.1)
