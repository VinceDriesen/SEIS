import math
import numpy as np
from .lidar import transform_lidar_scan

def update_occupancy_map(bot):
    """Update the occupancy map using the current LiDAR scan"""
    current_scan = np.array(bot.lidarSens.getRangeImage(), dtype=float)
    if len(current_scan) == 0:
        return
    
    # Get current robot pose
    robot_pos = (bot.position[0], bot.position[1])
    robot_yaw = bot.position[2]
    
    # Transform LiDAR points to world coordinates
    world_points = transform_lidar_scan(current_scan, robot_pos, robot_yaw)
    
    # Convert world coordinates to grid coordinates
    robot_grid_x = int(bot.position[0] * bot.map_resolution) + bot.map_offset
    robot_grid_y = int(bot.position[1] * bot.map_resolution) + bot.map_offset
    
    # Update occupancy map
    with bot.map_lock:
        # Mark robot position as free
        if 0 <= robot_grid_x < bot.grid_size and 0 <= robot_grid_y < bot.grid_size:
            bot.occupancy_map[robot_grid_y, robot_grid_x] = 0
            
        for point in world_points:
            grid_x = int(point[0] * bot.map_resolution) + bot.map_offset
            grid_y = int(point[1] * bot.map_resolution) + bot.map_offset
            
            if 0 <= grid_x < bot.grid_size and 0 <= grid_y < bot.grid_size:
                # Mark as occupied
                bot.occupancy_map[grid_y, grid_x] = 255
                
                # Draw line from robot to point (clear space in between)
                _draw_line(bot, robot_grid_x, robot_grid_y, grid_x, grid_y)
    
def _draw_line(bot, x0, y0, x1, y1):
    """Bresenham's line algorithm to mark free space between robot and obstacle"""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    while x0 != x1 or y0 != y1:
        if 0 <= x0 < bot.grid_size and 0 <= y0 < bot.grid_size:
            # Mark as free space (0-127 is free, with certainty increasing with lower values)
            # Don't overwrite occupied cells
            if bot.occupancy_map[y0, x0] != 255:
                bot.occupancy_map[y0, x0] = max(0, bot.occupancy_map[y0, x0] - 1)
        
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
            
            
def visualize_map(bot, save_path):
    """Visualize the current state of the occupancy map"""
    import matplotlib.pyplot as plt
    
    with bot.map_lock:
        plt.figure(figsize=(10, 10))
        plt.imshow(bot.occupancy_map, cmap='gray_r', origin='lower')
        
        # Mark robot position
        robot_x = int(bot.position[0] * bot.map_resolution) + bot.map_offset
        robot_y = int(bot.position[1] * bot.map_resolution) + bot.map_offset
        plt.plot(robot_x, robot_y, 'ro', markersize=10)
        
        # Draw direction indicator
        heading_len = 20
        dx = heading_len * math.cos(bot.position[2])
        dy = heading_len * math.sin(bot.position[2])
        plt.arrow(robot_x, robot_y, dx, dy, head_width=5, head_length=10, fc='r', ec='r')
        
        plt.title('Occupancy Map')
        plt.colorbar(label='Occupancy (0=free, 255=occupied)')
        plt.grid(True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()  # Close the figure to free memory