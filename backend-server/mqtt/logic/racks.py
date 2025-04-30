from typing import List, Tuple, Optional


class RackArea:
    def __init__(self, top_right: Tuple[float, float], bottom_right: Tuple[float, float],
                 top_left: Tuple[float, float], bottom_left: Tuple[float, float]):
        self.points = [top_right, bottom_right, top_left, bottom_left]
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        self.x_min = min(xs)
        self.x_max = max(xs)
        self.y_min = min(ys)
        self.y_max = max(ys)

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


class RackAreaMap:
    def __init__(self):
        self.areas: List[RackArea] = []

        raw_areas = [
            [(2.5, 3.7), (2.5, 3.65), (-2.5, 3.7), (-2.5, 3.65)],
            [(2.5, 3.65), (2.5, 2.85), (-2.5, 3.65), (-2.5, 2.85)],
            [(2.5, 2.85), (2.5, 2.8), (-2.5, 2.85), (-2.5, 2.8)],
            [(2.5, 2.0), (2.5, 1.95), (-2.5, 2.0), (-2.5, 1.95)],
            [(2.5, 1.95), (2.5, 1.15), (-2.5, 1.95), (-2.5, 1.15)],
            [(2.5, 1.15), (2.5, 1.1), (-2.5, 1.15), (-2.5, 1.1)],
            [(2.5, 0.3), (2.5, 0.25), (-2.5, 0.3), (-2.5, 0.25)],
            [(2.5, 0.25), (2.5, -0.55), (-2.5, 0.25), (-2.5, -0.55)],
            [(2.5, -0.55), (2.5, -0.5), (-2.5, -0.55), (-2.5, -0.5)],
            [(2.5, -1.3), (2.5, -1.35), (-2.5, -1.3), (-2.5, -1.35)],
            [(2.5, -1.35), (2.5, -2.15), (-2.5, -1.35), (-2.5, -2.15)],
            [(2.5, -2.15), (2.5, -2.2), (-2.5, -2.15), (-2.5, -2.2)],
            [(2.5, -3.0), (2.5, -3.05), (-2.5, -3.0), (-2.5, -3.05)],
            [(2.5, -3.05), (2.5, -3.85), (-2.5, -3.05), (-2.5, -3.85)],
            [(2.5, -3.85), (2.5, -3.9), (-2.5, -3.85), (-2.5, -3.9)],
        ]

        for area in raw_areas:
            self.areas.append(RackArea(*area))

    def find_area(self, x: float, y: float) -> Optional[int]:
        for i, area in enumerate(self.areas):
            if area.contains(x, y):
                return i
        return None

    def get_area(self, index: int) -> RackArea:
        return self.areas[index]


class RackAreaReservation:
    def __init__(self, rack_map: RackAreaMap):
        self.rack_map = rack_map
        self.occupancy: List[Optional[str]] = [None] * len(rack_map.areas)

    def reserve(self, index: int, robot_id: str) -> bool:
        """
        Reserve the rack area by robot_id.
        Returns True if successful, False otherwise.
        """
        if 0 <= index < len(self.occupancy) and self.occupancy[index] is None:
            self.occupancy[index] = robot_id
            return True
        return False

    def free_by_index(self, index: int) -> bool:
        """
        Free a reserved rack area by its index.
        Returns True if successful, False if area was already free.
        """
        if 0 <= index < len(self.occupancy) and self.occupancy[index] is not None:
            self.occupancy[index] = None
            return True
        return False

    def free_by_robot_id(self, robot_id: str) -> int:
        """
        Free all reserved areas for the given robot_id.
        Returns the number of freed areas.
        """
        count = 0
        for i in range(len(self.occupancy)):
            if self.occupancy[i] == robot_id:
                self.occupancy[i] = None
                count += 1
        return count  # Number of freed areas

    def is_reserved(self, index: int) -> bool:
        """
        Check if a rack area is reserved by any robot.
        """
        return self.occupancy[index] is not None

    def robot_at(self, index: int) -> Optional[str]:
        """
        Get the robot_id of the robot at the given index.
        """
        return self.occupancy[index]

    def reserve_at_position(self, x: float, y: float, robot_id: str) -> Optional[int]:
        """
        Reserve the area at the specified position for the robot.
        Returns the area index if successful, None otherwise.
        """
        area_index = self.rack_map.find_area(x, y)
        if area_index is not None and self.reserve(area_index, robot_id):
            return area_index
        return None
