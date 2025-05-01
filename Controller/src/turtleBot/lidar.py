from io import BytesIO
import base64
import math
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
        self.map_size = 5  # meters
        self.map_resolution = 0.015  # meters per cell
        self.grid_cells = int(self.map_size / self.map_resolution)
        self.grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.float32)
        self.free_prob = 0.4
        self.occ_prob = 0.7
        self.explored = -1
        self.exploration_radius = 0.35
        self.coverage_threshold = 0.9
        self.unknown_threshold_low = -0.5  # Log-odds
        self.unknown_threshold_high = 0.5  # Log-odds
        self.max_lidar_range_cells = int(
            3.5 / self.map_resolution
        )  # Max Lidar bereik in cellen (pas 3.5m aan indien nodig)

    def get_probability_grid(self):
        """Converteert log-odds grid naar probability grid (0=vrij, 1=bezet)."""
        return 1 / (1 + np.exp(-self.grid))  # Sigmoid

    def calculate_entropy(self, probability):
        """Berekent de entropie van een cel."""
        # Vermijd log(0) errors
        p = np.clip(probability, 1e-9, 1 - 1e-9)
        return -p * np.log2(p) - (1 - p) * np.log2(1 - p)

    def get_entropy_grid(self):
        """Berekent de entropie voor elke cel in de grid."""
        prob_grid = self.get_probability_grid()
        return self.calculate_entropy(prob_grid)

    def simulate_scan_from_pose(self, pose_grid_coords, pose_theta, num_rays=72):
        """
        Simuleert een Lidar scan vanaf een gegeven pose op de *huidige* grid.
        Retourneert een set van grid cellen die verwacht worden waargenomen.
        """
        observed_cells = set()
        fov_rad = math.radians(360)  # Lidar FOV (pas aan indien nodig)
        angle_increment = fov_rad / num_rays

        start_cell = tuple(pose_grid_coords)
        observed_cells.add(start_cell)

        # Huidige kansen op bezetting
        prob_grid = self.get_probability_grid()

        for i in range(num_rays):
            angle = pose_theta - fov_rad / 2 + angle_increment * i
            end_x = int(
                round(start_cell[0] + self.max_lidar_range_cells * math.cos(angle))
            )
            end_y = int(
                round(start_cell[1] + self.max_lidar_range_cells * math.sin(angle))
            )

            start_cell_int = (int(start_cell[0]), int(start_cell[1]))
            end_point_int = (int(end_x), int(end_y))

            line_cells = bresenham(start_cell_int, end_point_int)  # Jouw functie!

            for cell_coord in line_cells:
                r, c = cell_coord[0], cell_coord[1]

                if 0 <= r < self.grid_cells and 0 <= c < self.grid_cells:
                    cell_coords_tuple = (r, c)  # Zorg dat het een tuple is voor de set
                    observed_cells.add(cell_coords_tuple)
                    if prob_grid[r, c] > self.occ_prob:
                        break
                else:
                    break

        return list(
            observed_cells
        )  # Retourneer als lijst van tuples (r, c)urneer als lijst van tuples (r, c)

    def find_frontier_cells(self):
        """
        Vindt 'frontier' cellen: vrije cellen die grenzen aan onbekende cellen.
        Retourneert een lijst van (r, c) coördinaten van frontier cellen.
        """
        prob_grid = self.get_probability_grid()
        frontiers = []

        free_threshold = self.free_prob  # Cellen met kans < free_threshold zijn vrij

        is_unknown = (self.grid >= self.unknown_threshold_low) & (
            self.grid <= self.unknown_threshold_high
        )
        is_free = prob_grid < free_threshold

        # Vind vrije cellen
        free_indices = np.argwhere(is_free)

        # Check buren van vrije cellen
        for r, c in free_indices:
            # Check 8 buren
            is_frontier = False
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    # Check grenzen
                    if 0 <= nr < self.grid_cells and 0 <= nc < self.grid_cells:
                        if is_unknown[nr, nc]:
                            is_frontier = True
                            break
                if is_frontier:
                    break
            if is_frontier:
                frontiers.append((r, c))

        return frontiers

    def grid_to_world(self, grid_coords):
        """Converteert grid coördinaten (r, c) terug naar wereld coördinaten (x, y)."""

        world_x = (
            (grid_coords[0] * self.map_resolution)
            - self.map_size / 2
            + self.map_resolution / 2
        )  # Centrum van cel
        world_y = (
            (grid_coords[1] * self.map_resolution)
            - self.map_size / 2
            + self.map_resolution / 2
        )  # Centrum van cel

        world_x = (grid_coords[0] + 0.5) * self.map_resolution - self.map_size / 2
        world_y = (grid_coords[1] + 0.5) * self.map_resolution - self.map_size / 2

        return [world_x, world_y]

    def world_to_grid(self, world_coords):
        """
        Converts possibly out-of-bounds world (x,y) into valid grid (r,c).
        Clamps any infinities or NaNs to the nearest edge.
        """
        x, y = world_coords

        # 1) clamp to world extents
        half = self.map_size / 2
        if not math.isfinite(x):
            x = half if x > 0 else -half
        else:
            x = max(min(x,  half), -half)

        if not math.isfinite(y):
            y = half if y > 0 else -half
        else:
            y = max(min(y,  half), -half)

        # 2) convert to grid indices
        gx = (x + half) / self.map_resolution
        gy = (y + half) / self.map_resolution

        # 3) floor, int, and clip
        grid_x = int(np.floor(gx))
        grid_y = int(np.floor(gy))
        grid_x = np.clip(grid_x, 0, self.grid_cells - 1)
        grid_y = np.clip(grid_y, 0, self.grid_cells - 1)

        return np.array([grid_x, grid_y], dtype=int)
    def update_grid(self, sensor_pos, hits):
        """Werk de occupancy grid bij op basis van Lidar-metingen"""
        for hit in hits:
            line = bresenham(sensor_pos, hit)
            for cell in line[:-1]:
                self.update_cell(cell, self.free_prob)
            if line:
                self.update_cell(line[-1], self.occ_prob)

        for hit in hits:
            line = bresenham(sensor_pos, hit)
            log_odds = np.log(self.occ_prob / (1 - self.occ_prob))
            self.grid[line[-1][0], line[-1][1]] += log_odds
            self.grid = np.clip(self.grid, -10, 10)

    def update_cell(self, cell, probability):
        log_odds = np.log(probability / (1 - probability))
        self.grid[cell[0], cell[1]] += log_odds
        self.grid = np.clip(self.grid, -10, 10)

    def is_explored(self):
        prob_grid = 1 - 1 / (1 + np.exp(self.grid))
        known = (prob_grid <= 0.3) | (prob_grid >= 0.7)
        explored = np.sum(known)
        return explored / self.grid.size >= self.coverage_threshold
    
    def save_occupancy_map(self, filepath: str, log_odds: bool = False):
        """
        Save the current occupancy grid to an image file.
        
        :param filepath: Path (including filename) where the image will be saved.
        :param log_odds: If True, saves the raw log-odds grid; otherwise saves the probability grid.
        """
        if log_odds:
            data = self.grid
            cmap = 'RdBu'
            vmin, vmax = -5, 5
        else:
            data = self.get_probability_grid()
            cmap = 'viridis'
            vmin, vmax = 0.0, 1.0

        fig, ax = plt.subplots(figsize=(6, 6))
        extent = [
            -self.map_size / 2, self.map_size / 2,
            -self.map_size / 2, self.map_size / 2
        ]
        img = ax.imshow(
            data.T,
            origin='lower',
            extent=extent,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            interpolation='nearest'
        )
        ax.axis('off')  # Geen labels etc. als je dat wilt

        buf = BytesIO()
        fig.savefig(buf, format='png', dpi=150, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode('utf-8')
        return img_base64


class LidarFunctions:
    def __init__(self):
        self.occupancyGrid = OccupancyGrid()
        # self.visualization_thread = threading.Thread(target=self.visualize_grid, daemon=True)
        # self.visualization_thread.start()
        self.lidar_offset = -0.006

    def get_lidar_global_coord_values(self, lidarSensor: Lidar, position):
        """Verwerkt de Lidar-metingen en zet ze om naar globale coördinaten"""
        scan = lidarSensor.getRangeImage()
        fov = lidarSensor.getFov()
        angle_increment = fov / (lidarSensor.getHorizontalResolution() - 1)

        coords = []
        for i in range(lidarSensor.getHorizontalResolution()):
            angle = -fov / 2 + angle_increment * i
            distance = scan[i]
            if distance > 0:
                coords.append(self.local_to_global(distance, angle, position))
        return coords

    def local_to_global(self, distance, lidar_angle, position):
        theta = position["theta_value"]

        x_local = distance * math.cos(lidar_angle)
        y_local = distance * math.sin(lidar_angle)

        x_rot = x_local * math.cos(theta) + y_local * math.sin(theta)
        y_rot = -x_local * math.sin(theta) + y_local * math.cos(theta)

        x_global = x_rot + position["x_value"]
        y_global = y_rot - position["y_value"]

        return [x_global, -y_global]

    def get_robot_position_grid(self, position):
        """Berekent de sensorpositie in de occupancy grid op basis van de absolute wereldcoördinaten"""
        return self.occupancyGrid.world_to_grid(
            [position["x_value"], position["y_value"]]
        )

    def scan(self, lidarSensor: Lidar, position):
        """Voert een scan uit en werkt de occupancy grid bij"""
        # Transformeer lidar-metingen naar globale coördinaten
        sensor_pos = self.get_robot_position_grid(position)
        global_coords = self.get_lidar_global_coord_values(lidarSensor, position)

        # Update grid met gescande punten
        hits = [self.occupancyGrid.world_to_grid(c) for c in global_coords]
        self.occupancyGrid.update_grid(sensor_pos, hits)

    def visualize_grid(self):
        plt.ion()  # Interactieve modus
        fig, ax = plt.subplots(figsize=(10, 10))
        cmap = plt.cm.get_cmap("RdYlBu")
        norm = plt.Normalize(vmin=-2, vmax=2)

        # Initieel plot + colorbar buiten de loop
        grid_data = self.occupancyGrid.grid.T
        extent = [
            -self.occupancyGrid.map_size / 2,
            self.occupancyGrid.map_size / 2,
            -self.occupancyGrid.map_size / 2,
            self.occupancyGrid.map_size / 2,
        ]

        img = ax.imshow(
            grid_data,
            extent=extent,
            origin="lower",
            cmap=cmap,
            norm=norm,
            interpolation="nearest",
        )
        cbar = fig.colorbar(img, ax=ax)
        cbar.set_label("Log-Odds (Explored: <0, Occupied: >0)")

        ax.set_title("Hybride Occupancy Grid")
        ax.set_xlabel("X Positie (m)")
        ax.set_ylabel("Y Positie (m)")

        while True:
            grid_data = self.occupancyGrid.grid.T
            img.set_data(grid_data)
            img.set_clim(vmin=-2, vmax=2)  # Zorgt dat de kleuren consistent blijven
            plt.pause(0.1)

    def get_occupancy_grid(self):
        """
        Converteer log-odds NAAR JUISTE PROBABILITIES (zonder 1 - ...)
        """
        grid_prob = 1 / (
            1 + np.exp(-self.occupancyGrid.grid)
        )  # Dit is de correcte sigmoid
        extent = [
            -self.occupancyGrid.map_size / 2,
            self.occupancyGrid.map_size / 2,
            -self.occupancyGrid.map_size / 2,
            self.occupancyGrid.map_size / 2,
        ]
        return grid_prob.T, extent

    def save_occupancy_map(self, filepath: str = 'occupancy_map.png', log_odds: bool = False):
        return self.occupancyGrid.save_occupancy_map(filepath)