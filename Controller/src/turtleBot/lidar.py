import math
import numpy as np

def __lidar_scan_to_points(lidar_scan: np.ndarray):
    """Generates an x,y array of points based on the raw lidar scan.

    Args:
        lidar_scan (np.ndarray): Scan from the lidar sensor.

    Returns:
        np.ndarray: [x, y] array of all valid points in the robot's coordinate system.
    """
    points = []
    num_points = len(lidar_scan)
    
    # Webots LDS-01 specific configuration:
    # - Starts at back (180°) and rotates clockwise.
    # - Angular resolution = 2π / num_points.
    for i in range(num_points):
        angle = math.pi - i * (2 * math.pi / num_points)
        distance = lidar_scan[i]
        
        if distance == float('inf'):
            continue 
            
        # Convert to cartesian coordinates (robot's coordinate system)
        x = distance * math.cos(angle)  # Forward direction.
        y = distance * math.sin(angle)  # Left/right direction.
        points.append([x, y])
    
    return np.array(points)

def angle_robot_to_world(ori: tuple) -> float:
    """
    Calculate the robot's heading angle (in radians) from its orientation vector.
    
    Args:
        ori (tuple): A tuple (x, y) representing the robot's orientation vector 
                     (for example, (cos(θ), sin(θ))).
    
    Returns:
        float: The robot's orientation angle in radians.
    """
    # Using atan2 is more robust than manually checking the sign of y.
    return math.atan2(ori[1], ori[0])

def transform_lidar_scan(lidar_scan: np.ndarray, robot_pose: tuple, robot_ori: tuple) -> np.ndarray:
    """
    Transforms lidar scan points from the robot's local coordinate system to the world (map) coordinates.
    
    This function adapts the transformation logic from the original video example:
      - It first converts the raw lidar scan to (x, y) points (in the robot frame).
      - It then rotates these points based on the robot's orientation.
      - Finally, it translates them using the robot's position.
    
    Args:
        lidar_scan (np.ndarray): The raw lidar scan data (array of distances).
        robot_pose (tuple): The robot's position in the world as (x, y).
        robot_ori (tuple): The robot's orientation vector (e.g. (cos(θ), sin(θ))).
    
    Returns:
        np.ndarray: Transformed lidar points in world coordinates.
    """
    # Convert the lidar scan to points in the robot's frame.
    points = __lidar_scan_to_points(lidar_scan)
    
    # Compute the robot's heading angle (θ) in the world coordinate system.
    theta = angle_robot_to_world(robot_ori)
    
    # Create the rotation matrix to convert points from robot frame to world frame.
    rotMat = np.array([[np.cos(theta), -np.sin(theta)],
                       [np.sin(theta),  np.cos(theta)]])
    
    # Rotate the points and then add the robot's position to transform to world coordinates.
    points_map = np.dot(points, rotMat.T) + np.array(robot_pose)
    
    return points_map

def calculate_odometry_correction(current_scan: np.ndarray, previous_scan: np.ndarray):
    """Estimate delta position and rotation using ICP on two scans."""
    prev_points = __lidar_scan_to_points(previous_scan)
    curr_points = __lidar_scan_to_points(current_scan)
    
    if len(prev_points) == 0 or len(curr_points) == 0:
        return 0, 0, 0  # No valid points
    
    # Downsample for efficiency.
    prev_points = prev_points[::5]
    curr_points = curr_points[::5]
    
    # Initial transformation guess (identity).
    best_R = np.eye(2)
    best_t = np.zeros(2)
    best_error = np.inf
    max_iter = 20
    max_correspondence_dist = 0.5  # Tune this based on your environment.
    
    for _ in range(max_iter):
        # Find nearest neighbors with a distance check.
        correspondences = []
        for q in curr_points:
            distances = np.linalg.norm(prev_points - q, axis=1)
            min_idx = np.argmin(distances)
            if distances[min_idx] < max_correspondence_dist:
                correspondences.append((q, prev_points[min_idx]))
        
        # Early exit if not enough correspondences.
        if len(correspondences) < 10:
            break
            
        src = np.array([p for p, _ in correspondences])
        tgt = np.array([q for _, q in correspondences])
        
        # Compute centroids.
        src_centroid = np.mean(src, axis=0)
        tgt_centroid = np.mean(tgt, axis=0)
        
        # Center the points.
        src_centered = src - src_centroid
        tgt_centered = tgt - tgt_centroid
        
        # SVD-based transformation.
        H = src_centered.T @ tgt_centered
        U, _, Vt = np.linalg.svd(H)
        R_tmp = Vt.T @ U.T
        
        # Handle reflection.
        if np.linalg.det(R_tmp) < 0:
            Vt[1] *= -1
            R_tmp = Vt.T @ U.T
        
        t_tmp = tgt_centroid - (R_tmp @ src_centroid)
        
        # Calculate error between transformed source and target points.
        transformed_src = (R_tmp @ src.T).T + t_tmp
        error = np.mean(np.linalg.norm(transformed_src - tgt, axis=1))
        
        if error < best_error:
            best_R = R_tmp.copy()
            best_t = t_tmp.copy()
            best_error = error
    
    return best_t[0], best_t[1], np.arctan2(best_R[1, 0], best_R[0, 0])
