from typing import List, Tuple, Optional


class RackArea:
    def __init__(
        self,
        top_right: Tuple[float, float],
        bottom_right: Tuple[float, float],
        top_left: Tuple[float, float],
        bottom_left: Tuple[float, float],
    ):
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
        """Returns the index of the area, if the given position lies within one

        Args:
            x (float): x position
            y (float): y position

        Returns:
            Optional[int]: index of rack
        """
        for i, area in enumerate(self.areas):
            if area.contains(x, y):
                return i
        return None

    def get_area(self, index: int) -> RackArea:
        return self.areas[index]
