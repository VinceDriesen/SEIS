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

def transform_lidar_scan(lidar_scan: np.ndarray, robot_position: tuple, robot_yaw: float) -> np.ndarray:
    """
    Transforms lidar scan points using direct yaw angle instead of orientation vector.
    
    Args:
        lidar_scan: Raw lidar data
        robot_position: (x, y) tuple
        robot_yaw: Heading angle in radians
    
    Returns:
        Transformed points in world coordinates
    """
    points = __lidar_scan_to_points(lidar_scan)
    
    if len(points) == 0:
        return np.empty((0, 2))
    
    # Create rotation matrix from yaw angle
    c, s = np.cos(robot_yaw), np.sin(robot_yaw)
    rot_mat = np.array([[c, -s], [s, c]])
    
    # Rotate and translate points
    return points.dot(rot_mat.T) + np.array(robot_position)

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
