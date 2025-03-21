import math
from typing import List, Tuple
from controller import Lidar
import numpy as np
import matplotlib.pyplot as plt

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
        self.map_size = 10  # meters, -x to +x
        self.map_resolution = 0.05  # meters per cell
        self.grid_size = int(self.map_size / self.map_resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float16)
        self.free_prob = 0.4
        self.occ_prob = 0.7

    def coords_to_grid(self, coords):
        grid_x = int((coords[0] + self.map_size/2) / self.map_resolution)
        grid_y = int((coords[1] + self.map_size/2) / self.map_resolution)
        return np.clip([grid_x, grid_y], 0, self.grid_size-1).astype(int)

    def update_grid(self, sensor_pos, hits):
        for hit in hits:
            line = bresenham(sensor_pos, hit)
            for cell in line[:-1]:  # Alle cellen behalve de laatste
                self._update_cell(cell, self.free_prob)
            if line:  # Laatste cel als occupied markeren
                self._update_cell(line[-1], self.occ_prob)

    def _update_cell(self, cell, probability):
        x, y = int(cell[0]), int(cell[1])
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            log_odds = np.log(probability / (1 - probability))
            self.grid[x, y] += log_odds
            self.grid = np.clip(self.grid, -10, 10)

class LidarFunctions:
    def __init__(self):
        self.occupancyGrid = OccupancyGrid()
        
        
    def _get_sensor_grid_position(self, position: Tuple[float, float]):
        return self.occupancyGrid.coords_to_grid(position[:2])

    def scan_point_cloud(self, lidar: Lidar, position: List[float]):
        points_cloud = lidar.getPointCloud()
        robot_pos = position.copy()
        theta = robot_pos[2]
        global_coords = set()
        
        for point in points_cloud:
            if np.isnan(point.x) or np.isnan(point.y) or np.isinf(point.x) or np.isinf(point.y):
                continue
            rotated_x = point.x * math.cos(theta) - point.y * math.sin(theta)
            rotated_y = point.x * math.sin(theta) + point.y * math.cos(theta)
            
            # Translate to robot's world position
            global_x = robot_pos[0] + rotated_x
            global_y = robot_pos[1] + rotated_y
            global_coords.add((global_x, global_y))

        grid_coords = [self.occupancyGrid.coords_to_grid(c) for c in global_coords]
        grid_sensor = self._get_sensor_grid_position(robot_pos)
        
        self.occupancyGrid.update_grid(grid_sensor, grid_coords)
        self.visualize_grid()


    def visualize_grid(self):
        plt.clf()
        grid_prob = 1 - 1/(1 + np.exp(self.occupancyGrid.grid))  # Log-odds naar probability
        extent = [-self.occupancyGrid.map_size/2, self.occupancyGrid.map_size/2, -self.occupancyGrid.map_size/2, self.occupancyGrid.map_size/2]
        plt.imshow(grid_prob.T, origin='lower', cmap='binary', vmin=0, vmax=1, extent=extent)
        plt.colorbar(label='Occupancy Probability')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.savefig('./occupancy_grid.png')