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

STATE_IDLE = 'idle'
ACTION_MOVING = 'moving'
ACTION_ROTATING = 'rotating'
ACTION_SCANNING = 'scanning'

TASK_MOVE_TO = 'move_to'
TASK_ROTATE = 'rotate'
TASK_EXPLORE = 'explore'
TASK_MOVE_REL = 'move_rel'
TASK_ROTATE_REL = 'rotate_rel'

MOVE_TO_STATE_PLANNING = 'planning_move'
MOVE_TO_STATE_ROTATION = 'planning_rotation'
MOVE_TO_STATE_MOVING = 'planning_moving'
MOVE_TO_STATE_FINISHED = 'planning_finished'
MOVE_TO_STATE_FAILED = 'planning_failed'

EXPLORE_STATE_START = 'explore_start'
EXPLORE_STATE_SCANNING = 'explore_scanning'
EXPLORE_STATE_PLANNING = 'explore_planning'
EXPLORE_STATE_MOVING_TO_FRONTIER = 'explore_moving_to_frontier'
EXPLORE_STATE_ROTATING_TO_FRONTIER = 'explore_rotating_to_frontier'
EXPLORE_STATE_FINISHED = 'explore_finished'
EXPLORE_STATE_STUCK = 'explore_stuck'

MOVEMENT_TOLERANCE = 0.1
ROTATION_TOLERANCE_RAD = math.radians(2.0)

class TurtleBotSM:
    """
    TurtleBot State Machine class. This class is responsible for managing the state machine of the TurtleBot.
    It handles the robot's sensors, motors, and state transitions.
    """

    def __init__(self, name: str, robot: Robot, time_step :float, max_speed: float):
        self.name = name
        self.robot = robot
        self.time_step = time_step
        self.max_speed = max_speed
        self.left_motor: Motor = self.robot.getDevice("left wheel motor")
        self.right_motor: Motor = self.robot.getDevice("right wheel motor")
        self.radius = 0.033
        self.distance_between_wheels = 0.1775
        self.firstRun = True
        
        self._enableSensors()
        
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)        
            
        # Init alle variables voor de state machine, ik krijg hier echt depressie van hoor
        self._current_action: str = STATE_IDLE
        self._current_task_details: dict | None = None
        self.position = [0.0, 0.0, 0.0]
        self._prev_left_encoder = 0.0
        self._prev_right_encoder = 0.0
        self._is_first_update = True
        
        self._last_grid_shape = None
        self._last_extent = None
        
        self._move_target_distance = 0.0
        self._move_start_encoder = 0.0
        self._rotate_target_angle_rad = 0.0
        
        self._move_to_target_pos = [0.0, 0.0]
        self._move_to_path: list | None = None
        self._move_to_path_index = 0
        self._move_to_state: str | None = None
        
        self._explore_state: str | None = None 
        self._explore_processed_targets: set = set()
        self._explore_tries = 0 
        self._explore_consecutive_pathfinding_failures = 0 
        self._explore_stagnation_count = 0 
        self._explore_num_candidates = 30
        self._explore_cost_alpha = 1.0
        self._explore_max_consecutive_failures = 10
        self._explore_max_stagnation_iterations = 5
        
        self.velocity_norm = 0.3
        self.nonMeasuredPosition = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        
        #Init de Lidar
        try:
            self.lidar = LidarFunctions()
        except NameError:
            print("LidarFunctions class not found. Please check the import statement.")
            self.lidar = None
            
        
        
    def _initalizeSub(self):
        print("Initializing special sub!")
        
        # GPS dingen
        gps_values = self.gps.getValues()
        if gps_values:
            self.position[0] = gps_values[0]
            self.position[1] = gps_values[1]
            compass_val = self.getHeadingFromCompass()
            if compass_val:
                self.position[2] = compass_val
        print(f"Robot {self.name}: Initial estimated position: {self.getGpsPosition()}")
            
            
    def _enableSensors(self):
        """Enables all sensors."""
        self.frontDistSens: DistanceSensor = self.robot.getDevice("front distance sensor")
        self.rearDistSens: DistanceSensor = self.robot.getDevice("rear distance sensor")
        self.leftDistSens: DistanceSensor = self.robot.getDevice("left distance sensor")
        self.rightDistSens: DistanceSensor = self.robot.getDevice("right distance sensor")
        self.leftMotorSens: PositionSensor = self.robot.getDevice("left wheel sensor")
        self.rightMotorSens: PositionSensor = self.robot.getDevice("right wheel sensor")

        self.lidarSens: Lidar = self.robot.getDevice("LDS-01")
        self.lidarMotor1: Motor = self.robot.getDevice("LDS-01_main_motor")
        self.lidarMotor2: Motor = self.robot.getDevice("LDS-01_secondary_motor")

        self.compass: Compass = self.robot.getDevice("compass")
        self.gps: GPS = self.robot.getDevice("gps")

        self.frontDistSens.enable(self.time_step)
        self.rearDistSens.enable(self.time_step)
        self.leftDistSens.enable(self.time_step)
        self.rightDistSens.enable(self.time_step)
        self.leftMotorSens.enable(self.time_step)
        self.rightMotorSens.enable(self.time_step)
        self.lidarSens.enable(self.time_step)
        self.compass.enable(self.time_step)
        self.gps.enable(self.time_step)
        
    def getHeadingFromCompass(self) -> float | None:
        """
        Returns the heading of the robot in radians.
        """
        if not self.compass:
            print("Compass device not initialized.")
            return None
        
        x_c, y_c, _ = self.compass.getValues()
        heading = math.atan2(x_c, y_c)
        return self.normAngle(heading)
    
    def normAngle(self, angle: float) -> float:
        return angle % (2 * math.pi)
    
    def getGpsPosition(self) -> list[str, float]:
        """
        Returns the GPS position of the robot.
        """
        return {
            'x_value': self.position[0],
            'y_value': self.position[1],
            'theta_value': self.position[2]
        }
        # return self.gps.getValues()
        
    def isBusy(self) -> bool:
        """
        Returns True if the robot is busy, False otherwise.
        """
        return self._current_action != STATE_IDLE
    
    def _taskMoveTo(self, target_pos: list[float]) -> bool:
        x, y = target_pos
        if x is not None and y is not None:
            self._move_to_target_pos = [x, y]
            self._current_action = TASK_MOVE_TO
            self._move_to_state = MOVE_TO_STATE_PLANNING
            print(f"Robot {self.name}: Initiated move_to task to ({x:.2f}, {y:.2f})")
            return True
        else:
            print(f"Robot {self.name} ERROR: '{TASK_MOVE_TO}' task requires 'x' and 'y' parameters.")
            self._current_action = STATE_IDLE
            return False
        
    def _taskRotateRel(self, angle_deg: float) -> bool:
        if angle_deg is not None:
            current_heading = self.position[2]
            target_heading_rad = self.normAngle(current_heading + math.radians(angle_deg))
            self._rotate_target_angle_rad = target_heading_rad 
            self._current_action = ACTION_ROTATING 
            self.startRotating(target_heading_rad)
            print(f"Robot {self.name}: Initiated rotate_relative task by {angle_deg:.2f} deg")
            return True
        else:
            print(f"Robot {self.name} ERROR: '{TASK_ROTATE_REL}' task requires 'angle' parameter.")
            self._current_action = STATE_IDLE
            return False
        
    def _taskMoveRel(self, dx: float, dy: float) -> bool:
        current_pos = self.position
        target_x = current_pos[0] + dx * math.cos(current_pos[2]) - dy * math.sin(current_pos[2])
        target_y = current_pos[1] + dx * math.sin(current_pos[2]) + dy * math.cos(current_pos[2])
        self._move_to_target_pos = [target_x, target_y]
        self._current_action = TASK_MOVE_TO
        self._move_to_state = MOVE_TO_STATE_PLANNING
        self.start
        print(f"Robot {self.name}: Initiated move_relative task to ({target_x:.2f}, {target_y:.2f})")
        return True
        
    def executeTask(self, task: dict) -> bool:
        """
        Called by the scheduler to assign a new task.
        This method validates the task and INITIATES it by setting internal state.
        Returns True if the task type is recognized and initiation starts, False otherwise.
        The actual task execution happens step-by-step via update_task_execution().
        """
        
        if self.isBusy():
            print(f"Robot {self.name}: Cannot start task {task.get('type', 'Unknown')}, already busy with {self._current_action}")
            return False
        
        task_type = task.get('type')
        task_params = task.get('params', {})
        print(f"Robot {self.name}: Received task: {task_type} with params: {task_params}")
        
        self._current_task_details = task
        self._move_to_state = None
        self._move_to_path = None
        self._move_to_path_index = 0
        self._explore_state = None
        
        if task_type == TASK_MOVE_TO:
            x, y = task_params.get('x'), task_params.get('y')
            return self._taskMoveTo([x, y])
        elif task_type == TASK_ROTATE_REL:
            angle = task_params.get('angle')
            return self._taskRotateRel(angle)
        elif task_type == TASK_MOVE_REL:
            dx, dy = task_params.get('x', 0.0), task_params.get('y', 0.0)
            return self._taskMoveRel(dx, dy)
        elif task_type == TASK_EXPLORE:
            self._explore_num_candidates = task_params.get('num_candidates', 30)
            self._explore_cost_alpha = task_params.get('cost_alpha', 1.0)
            self._explore_max_consecutive_failures = task_params.get('max_consecutive_failures', 10)
            self._explore_max_stagnation_iterations = task_params.get('max_stagnation_iterations', 5)

            self._current_action = TASK_EXPLORE
            self._explore_state = EXPLORE_STATE_START
            print(f"Robot {self.name}: Initiated explore task")
            return True
        else:
            print(f"Robot {self.name} ERROR: Unknown task type '{task_type}'")
            self._current_action = STATE_IDLE
            self._current_task_details = None
            return False
        
    def updateTaskExecution(self) -> bool:
        """
        Called by the main Webots loop every time step.
        Performs odometry, updates sensor readings (implicitly via robot.step),
        and executes one step of the current action's state machine.
        Returns True if a task is ongoing, False if idle.
        """
        
        if self._is_first_update:
            self._initalizeSub()
            self._prev_left_encoder = self.leftMotorSens.getValue()
            self._prev_right_encoder = self.rightMotorSens.getValue()
            self._is_first_update = False
            return self._current_action != STATE_IDLE
        
        current_left_encoder = self.leftMotorSens.getValue()
        current_right_encoder = self.rightMotorSens.getValue()
        
        delta_left_encoder = current_left_encoder - self._prev_left_encoder
        delta_right_encoder = current_right_encoder - self._prev_right_encoder
        
        self._prev_left_encoder = current_left_encoder
        self._prev_right_encoder = current_right_encoder
        
        delta_distance_linear = ((delta_left_encoder * self.radius) + (delta_right_encoder * self.radius)) / 2.0
        delta_theta_odometry = ((delta_right_encoder * self.radius) - (delta_left_encoder * self.radius)) / self.distance_between_wheels
        
        heading_for_odometry = self.position[2]
        self.position[0] += delta_distance_linear * math.cos(heading_for_odometry)
        self.position[1] += delta_distance_linear * math.sin(heading_for_odometry)
        
        compass_heading = self.getHeadingFromCompass()
        if compass_heading is not None:
            self.position[2] = compass_heading
        else:
            self.position[2] = self.normAngle(self.position[2] + delta_theta_odometry)
            
        action_ongoing = False
        
        if self._current_action == STATE_IDLE:
            return False

        elif self._current_action == ACTION_MOVING:
            action_ongoing = self._updateMovingAction()

        elif self._current_action == ACTION_ROTATING:
            action_ongoing = self._updateRotatingAction()

        elif self._current_action == TASK_MOVE_TO:
            action_ongoing = self._updateMoveToAction()

        elif self._current_action == TASK_EXPLORE:
            action_ongoing = self._updateExploreAction()
            
            
        if self._current_action != STATE_IDLE and not action_ongoing:
            finished_action = self._current_action 
            self._current_action = STATE_IDLE 
            self._current_task_details = None
            print(f"Robot {self.name}: Action '{finished_action}' finished.")
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)


        return self._current_action != STATE_IDLE
    
    
    def startMoving(self, distance: float):
        """Initiates a linear movement. Requires update_task_execution calls."""
        if self._current_action != STATE_IDLE: return False
        
        print(f"Robot {self.name}: Starting linear movement of {distance:.2f} m")
        self._current_action = ACTION_MOVING
        self._move_to_distance = distance
        self._move_start_encoder = self.leftMotorSens.getValue()
        
        lineare_velocity = self.max_speed
        if distance < 0:
            lineare_velocity *= -1
            
        self.left_motor.setVelocity(lineare_velocity)
        self.right_motor.setVelocity(lineare_velocity)
        
    def _updateMovingAction(self) -> bool:
        if self._current_action != ACTION_MOVING: return False 

        current_encoder = self.leftMotorSens.getValue()
        current_distance_travelled = (current_encoder - self._move_start_encoder) * self.radius

        target_reached = False
        if self._move_target_distance > 0 and current_distance_travelled >= self._move_target_distance - MOVEMENT_TOLERANCE:
             target_reached = True
        elif self._move_target_distance < 0 and current_distance_travelled <= self._move_target_distance + MOVEMENT_TOLERANCE:
             target_reached = True

        if target_reached:
            print(f"Robot {self.name}: Movement finished. Travelled {current_distance_travelled:.2f}/{self._move_target_distance:.2f}m")
            return False 

        return True 
    
    def startRotating(self, target_heading_rad: float):
        """Initiates rotation to an absolute heading. Requires update_task_execution calls."""
        if self._current_action != STATE_IDLE: return False # Should be checked before calling

        print(f"Robot {self.name}: Starting rotate to {math.degrees(target_heading_rad):.2f} deg (rad: {target_heading_rad:.2f})")
        self._current_action = ACTION_ROTATING
        self._rotate_target_angle_rad = self.normAngle(target_heading_rad)

        current_heading = self.position[2]
        angle_needed = self.normAngle(self._rotate_target_angle_rad - current_heading)

        angle_velocity = self.velocity_norm * self.max_speed
        if angle_needed >= 0: # Prefer CCW for positive angle_needed
            self.left_motor.setVelocity(-angle_velocity)
            self.right_motor.setVelocity(angle_velocity)
        else: # Prefer CW for negative angle_needed
            self.left_motor.setVelocity(angle_velocity)
            self.right_motor.setVelocity(-angle_velocity)



    def _updateRotatingAction(self) -> bool:
        """Updates rotation progress. Returns True if ongoing."""
        if self._current_action != ACTION_ROTATING: return False # Should be in the correct state

        current_heading = self.position[2] # Use current estimated heading

        error = self.normalizeAngle(self._rotate_target_angle_rad - current_heading)

        if abs(error) <= ROTATION_TOLERANCE_RAD:
             print(f"Robot {self.name}: Rotation finished. Current heading {math.degrees(current_heading):.2f}, Target {math.degrees(self._rotate_target_angle_rad):.2f}")
             return False # Indicate finished

        return True # Indicate ongoing
    
    def startExploration(self, params: dict):
        """Initiates the exploration task. Requires update_task_execution calls."""
        if self._current_action != STATE_IDLE: return False

        print(f"Robot {self.name}: Starting exploration task.")
        self._current_action = TASK_EXPLORE
        self._explore_num_candidates = params.get('num_candidates', 30)
        self._explore_cost_alpha = params.get('cost_alpha', 1.0)
        self._explore_max_consecutive_failures = params.get('max_consecutive_failures', 10)
        self._explore_max_stagnation_iterations = params.get('max_stagnation_iterations', 5)

        self._explore_state = EXPLORE_STATE_START # Start the state machine
        self._explore_processed_targets = set()
        self._explore_tries = 0 # Used in original, needs integration
        self._explore_consecutive_pathfinding_failures = 0
        self._explore_stagnation_count = 0


        return True # Indicate initiation started
    
    def _updateExploreAction(self) -> bool:
        if self._current_action != TASK_EXPLORE: return False

        if self._explore_state == EXPLORE_STATE_START:
            self._explore_state = EXPLORE_STATE_SCANNING
            return True

        elif self._explore_state == EXPLORE_STATE_SCANNING:
            current_pos_dict = self.getGpsPosition()
            self.lidar.scan(self.lidarSens, current_pos_dict)

            if self.lidar.occupancyGrid.is_explored():
                self._explore_state = EXPLORE_STATE_FINISHED
                return False

            self._explore_state = EXPLORE_STATE_PLANNING
            return True

        elif self._explore_state == EXPLORE_STATE_PLANNING:
            grid_obj = self.lidar.occupancyGrid
            current_pos_dict = self.getGpsPosition()
            current_theta = current_pos_dict['theta_value']

            frontier_cells_rc = grid_obj.find_frontier_cells()

            if not frontier_cells_rc:
                if self.lidar.occupancyGrid.is_explored():
                    self._explore_state = EXPLORE_STATE_FINISHED
                    return False
                else:
                    self._explore_state = EXPLORE_STATE_STUCK
                    return False

            candidate_info = []
            entropy_grid = grid_obj.get_entropy_grid()

            candidate_grid_cells_rc = frontier_cells_rc
            if len(frontier_cells_rc) > self._explore_num_candidates:
                indices = np.random.choice(len(frontier_cells_rc), self._explore_num_candidates, replace=False)
                candidate_grid_cells_rc = [frontier_cells_rc[i] for i in indices]

            for candidate_cell_rc in candidate_grid_cells_rc:
                observed_cells_indices_rc = grid_obj.simulate_scan_from_pose(candidate_cell_rc, current_theta)
                information_gain = 0
                for r, c in observed_cells_indices_rc:
                    if 0 <= r < grid_obj.grid_cells and 0 <= c < grid_obj.grid_cells:
                        information_gain += entropy_grid[r, c]

                candidate_world_coords = grid_obj.grid_to_world(candidate_cell_rc)
                dx = candidate_world_coords[0] - current_pos_dict['x_value']
                dy = candidate_world_coords[1] - current_pos_dict['y_value']
                cost = math.sqrt(dx**2 + dy**2)

                utility = information_gain / (cost ** self._explore_cost_alpha) if cost > 1e-6 else information_gain

                candidate_info.append({
                    'cell_rc': candidate_cell_rc,
                    'world': candidate_world_coords,
                    'gain': information_gain,
                    'cost': cost,
                    'utility': utility
                })

            if not candidate_info:
                self.stagnation_count += 1
                if self.stagnation_count >= self._explore_max_stagnation_iterations:
                    self._explore_state = EXPLORE_STATE_STUCK
                    return False
                else:
                    self._explore_state = EXPLORE_STATE_SCANNING
                    return True

            candidate_info.sort(key=lambda x: x['utility'], reverse=True)

            selected_candidate = None
            for candidate in candidate_info:
                target_key = (round(candidate['world'][0], 2), round(candidate['world'][1], 2))
                if target_key not in self._explore_processed_targets:
                    selected_candidate = candidate
                    self._explore_processed_targets.add(target_key)
                    break

            if selected_candidate:
                target_world_pos = selected_candidate['world']
                print(f"Robot {self.name}: Planned move to frontier {target_world_pos}. Transitioning to move.")

                self._current_action = TASK_MOVE_TO # <<< ZET HIER DE HOOFD ACTIE >>>
                move_init_success = self.startMoveTo(target_world_pos[0], target_world_pos[1], internal_call=True)

                if move_init_success:
                    self._explore_state = EXPLORE_STATE_MOVING_TO_FRONTIER # Explore state blijft deze toestand
                    return True # Explore is nog bezig (in de move fase)
                else:
                    print(f"Robot {self.name}: ERROR: Failed to initiate internal move_to.")
                    self._explore_consecutive_pathfinding_failures += 1
                    if self._explore_consecutive_pathfinding_failures >= self._explore_max_consecutive_failures:
                        self._explore_state = EXPLORE_STATE_STUCK
                        return False
                    else:
                        self._explore_state = EXPLORE_STATE_SCANNING # Terug naar scannen/plannen na fout
                        return True
            else:
                self.stagnation_count += 1
                if self.stagnation_count >= self._explore_max_stagnation_iterations:
                    self._explore_state = EXPLORE_STATE_STUCK
                    return False
                else:
                    self._explore_state = EXPLORE_STATE_SCANNING
                    return True

        elif self._explore_state == EXPLORE_STATE_MOVING_TO_FRONTIER:
            if self._current_action == TASK_MOVE_TO:
                pass
            elif self._current_action == STATE_IDLE:
                if hasattr(self, '_move_to_success') and self._move_to_success:
                    print(f"Robot {self.name}: Successfully moved to frontier. Resuming exploration.")
                    self.stagnation_count = 0 # Reset stagnation
                    self._explore_consecutive_pathfinding_failures = 0 # Reset failures
                    self._explore_state = EXPLORE_STATE_SCANNING # Ga naar volgende ronde van explore
                else:
                    print(f"Robot {self.name}: Failed to move to frontier. Handling failure.")
                    self._explore_consecutive_pathfinding_failures += 1
                    if self._explore_consecutive_pathfinding_failures >= self._explore_max_consecutive_failures:
                        self._explore_state = EXPLORE_STATE_STUCK
                    else:
                        print(f"Robot {self.name}: Consecutive pathfinding failures: {self._explore_consecutive_pathfinding_failures}. Replanning/Rescanning.")
                        self._explore_state = EXPLORE_STATE_PLANNING
                return True

        elif self._explore_state == EXPLORE_STATE_STUCK:
            self._explore_state = EXPLORE_STATE_FINISHED
            return False

        elif self._explore_state == EXPLORE_STATE_FINISHED:
            return False

        else:
            self._explore_state = EXPLORE_STATE_STUCK
            return False
    
    
    def startMoveTo(self, x: float, y: float, internal_call: bool = False) -> bool:
        """Initiates a move_to task using pathfinding. Requires update_task_execution calls."""
        if not internal_call and self._current_action != STATE_IDLE:
            print(f"Robot {self.name}: Cannot start move_to, already busy with {self._current_action}")
            return False
         
        if not internal_call and self._current_action == TASK_MOVE_TO:
             print(f"Robot {self.name}: Warning: explore is trying to start move_to, but action is already MOVE_TO. Ignoring?")
             return False

        print(f"Robot {self.name}: Starting move_to task to ({x:.2f}, {y:.2f})")
        
        if not internal_call and self._current_action != STATE_IDLE:
            print(f"Robot {self.name}: Cannot start move_to externally, already busy with {self._current_action}")
            return False
        
        if internal_call and self._current_action == TASK_MOVE_TO:
            pass # Complexiteit uitgesteld

        if not internal_call and self._current_action != STATE_IDLE:
             print(f"Robot {self.name}: Cannot start move_to externally, already busy with {self._current_action}")
             return False


        self._move_to_target_pos = [x, y]
        self._move_to_state = MOVE_TO_STATE_PLANNING # Zet init state voor move_to state machine
        self._move_to_path = None
        self._move_to_path_index = 0
        self._move_to_success = False # Reset succes vlag

        return True
    
    def _updateMoveToAction(self) -> bool:
        """Updates the state machine for a move_to task. Returns True if ongoing."""
        if self._current_action != TASK_MOVE_TO: return False

        if self._move_to_state == MOVE_TO_STATE_PLANNING:
            current_pos = self.getGpsPosition()
            target_pos = self._move_to_target_pos

            grid_prob, extent = self.lidar.get_occupancy_grid()
            binary_grid = 1 - (grid_prob > 0.8).astype(np.int8)
            buffered_grid_np = self.addBufferToGrid(binary_grid, extent)

            def real_to_grid(real_x, real_y, grid_shape, grid_extent):
                grid_width = grid_shape[1]
                grid_height = grid_shape[0]
                grid_x = int((real_x - grid_extent[0]) / (grid_extent[1]-grid_extent[0]) * grid_width)
                grid_y = int((real_y - grid_extent[2]) / (grid_extent[3]-grid_extent[2]) * grid_height)
                return np.clip(grid_x, 0, grid_width - 1), np.clip(grid_y, 0, grid_height - 1)

            start_x_grid, start_y_grid = real_to_grid(current_pos['x_value'], current_pos['y_value'], buffered_grid_np.shape, extent)
            end_x_grid, end_y_grid = real_to_grid(target_pos[0], target_pos[1], buffered_grid_np.shape, extent)

            if buffered_grid_np[end_y_grid, end_x_grid] == 0:
                self._move_to_state = MOVE_TO_STATE_FAILED
                return False

            try:
                grid = Grid(matrix=buffered_grid_np.tolist())
                start_node = grid.node(start_x_grid, start_y_grid)
                end_node = grid.node(end_x_grid, end_y_grid)

                if not start_node.walkable:
                    self._move_to_state = MOVE_TO_STATE_FAILED
                    return False
                if not end_node.walkable:
                    self._move_to_state = MOVE_TO_STATE_FAILED
                    return False

                finder = AStarFinder()
                path, runs = finder.find_path(start_node, end_node, grid)

                if path:
                    simplified_path = self.simplifyPath(path, grid)
                    if simplified_path:
                        self._move_to_path = simplified_path
                        self._move_to_path_index = 0
                        self._move_to_state = MOVE_TO_STATE_MOVING
                        
                        self._last_grid_shape = buffered_grid_np.shape
                        self._last_extent = extent
                        return True
                    else:
                        self._move_to_state = MOVE_TO_STATE_FAILED
                    return False
                else:
                    self._move_to_state = MOVE_TO_STATE_FAILED
                    return False

            except Exception as e:
                print(f"Error during move_to planning: {e}")
                self._move_to_state = MOVE_TO_STATE_FAILED
                return False

        elif self._move_to_state == MOVE_TO_STATE_MOVING:
            current_pos = self.getGpsPosition()
            next_waypoint_grid = self._move_to_path[self._move_to_path_index + 1]

            if not hasattr(self, '_last_grid_shape') or self._last_grid_shape is None:
                self._move_to_state = MOVE_TO_STATE_FAILED
                return False

            next_waypoint_world_x = (next_waypoint_grid.x) / (self._last_grid_shape[1]) * (self._last_extent[1] - self._last_extent[0]) + self._last_extent[0]
            next_waypoint_world_y = (next_waypoint_grid.y) / (self._last_grid_shape[0]) * (self._last_extent[3] - self._last_extent[2]) + self._last_extent[2]

            dist_to_next_waypoint = math.sqrt(
                (current_pos['x_value'] - next_waypoint_world_x)**2 +
                (current_pos['y_value'] - next_waypoint_world_y)**2
            )

            WAYPOINT_REACH_TOLERANCE = 0.03
            if dist_to_next_waypoint < WAYPOINT_REACH_TOLERANCE:
                self._move_to_path_index += 1

            if self._move_to_path_index >= len(self._move_to_path) - 1:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                self._move_to_state = MOVE_TO_STATE_FINISHED
                return False

            current_waypoint_world = [current_pos['x_value'], current_pos['y_value']]
            target_waypoint_world = [next_waypoint_world_x, next_waypoint_world_y]

            dx = target_waypoint_world[0] - current_waypoint_world[0]
            dy = target_waypoint_world[1] - current_waypoint_world[1]
            distance_to_waypoint = math.sqrt(dx**2 + dy**2)
            angle_to_waypoint_rad = math.atan2(dy, dx)

            current_heading = self.position[2]
            angle_needed_rel = self.normAngle(angle_to_waypoint_rad - current_heading)

            k_rot = 3
            k_lin = 10.0

            if abs(angle_needed_rel) > ROTATION_TOLERANCE_RAD * 2:
                v_left = -math.copysign(self.velocity_norm * self.max_speed, angle_needed_rel)
                v_right = math.copysign(self.velocity_norm * self.max_speed, angle_needed_rel)
            else:
                forward_speed = min(self.velocity_norm * self.max_speed, k_lin * distance_to_waypoint)
                turn_speed = k_rot * angle_needed_rel

                v_left = forward_speed - turn_speed
                v_right = forward_speed + turn_speed

                max_wheel_speed = self.velocity_norm * self.max_speed
                v_left = max(-max_wheel_speed, min(max_wheel_speed, v_left))
                v_right = max(-max_wheel_speed, min(max_wheel_speed, v_right))

            self.left_motor.setVelocity(v_left)
            self.right_motor.setVelocity(v_right)
            
            print(f"Robot {self.name}: Moving to waypoint {self._move_to_path_index + 1}/{len(self._move_to_path) - 1} at ({target_waypoint_world[0]:.2f}, {target_waypoint_world[1]:.2f}), distance: {distance_to_waypoint:.2f}, angle: {math.degrees(angle_needed_rel):.2f} deg, velocity: ({v_left:.2f}, {v_right:.2f})")

            return True

        elif self._move_to_state == MOVE_TO_STATE_FINISHED:
            return False

        elif self._move_to_state == MOVE_TO_STATE_FAILED:
            return False

        else:
             self._move_to_state = MOVE_TO_STATE_FAILED
             return False

    def addBufferToGrid(self, grid, extent, buffer_distance=0.2):
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
    
    def visPathfindingGrid(self, grid, path=None, start=None, end=None):
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
        
    def bresenhamLine(self, x0, y0, x1, y1):
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
    
    def simplifyPath(self, path, grid):
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
            for next_index in range(len(path)-1, current_index, -1):
                if self.hasLineOfSight(path[current_index], path[next_index], grid):
                    furthest_index = next_index
                    break
            simplified.append(path[furthest_index])
            current_index = furthest_index

        return simplified

    def hasLineOfSight(self, node_a, node_b, grid):
        """Controleer of er een rechte lijn is tussen twee nodes zonder obstakels."""
        x0, y0 = node_a.x, node_a.y
        x1, y1 = node_b.x, node_b.y
        line = self.bresenhamLine(x0, y0, x1, y1)

        for (x, y) in line:
            if x < 0 or y < 0 or x >= grid.width or y >= grid.height:
                return False
            if not grid.node(x, y).walkable:
                return False
        return True

