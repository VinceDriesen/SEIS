import math
import numpy as np


def transform_lidar_scan(lidar_scan: np.ndarray, robot_position: tuple, robot_yaw: float) -> np.ndarray:
    """
    Transforms lidar scan points from robot coordinates to world coordinates.
    """
    robot_points = []
    num_points = len(lidar_scan)
    
    for i in range(num_points):
        # Correct clockwise rotation with negative angle
        angle = -i * (2 * math.pi / num_points)  # Key fix: negative for clockwise
        distance = lidar_scan[i]
        
        if distance == float('inf') or distance <= 0:
            continue
            
        # Robot frame: +x forward, +y left
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        robot_points.append([x, y])
    
    if not robot_points:
        return np.empty((0, 2))
    
    robot_points = np.array(robot_points)
    
    # Create rotation matrix
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    rotation_matrix = np.array([
        [cos_yaw, -sin_yaw],
        [sin_yaw, cos_yaw]
    ])
    
    # Correct rotation: use rotation_matrix (not .T)
    world_points = np.dot(robot_points, rotation_matrix) + robot_position
    
    return world_points