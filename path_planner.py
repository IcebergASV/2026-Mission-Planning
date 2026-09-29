import heapq
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from coordinate_utils import lat_lon_to_local, local_to_lat_lon, euclidean_distance


@dataclass
class PlannerConfig:
    grid_resolution_m: float = 1.0      # meters per grid cell
    inflation_radius_m: float = 3.0     # obstacle inflation in meters
    area_width_m: float = 320.0         # competition area width
    area_height_m: float = 160.0        # competition area height


@dataclass(order=True)
class Node:
    f_score: float                        # priority for heap
    g_score: float = field(compare=False) # cost from start
    row: int = field(compare=False)
    col: int = field(compare=False)
    parent: Optional[tuple] = field(compare=False, default=None)


class PathPlanner:

    def __init__(self, config: PlannerConfig, origin_lat: float, origin_lon: float):
        self.config = config
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon

        # grid dimensions
        self.rows = int(config.area_height_m / config.grid_resolution_m)
        self.cols = int(config.area_width_m / config.grid_resolution_m)

        # occupancy grid — False = free, True = occupied
        self.grid = np.zeros((self.rows, self.cols), dtype=bool)

    def update_obstacles(self, obstacles: list[tuple[float, float]]):
        """
        Takes a list of (lat, lon) obstacle positions.
        Marks them and their inflation radius on the grid.
        """
        # clear grid first
        self.grid = np.zeros((self.rows, self.cols), dtype=bool)

        inflation_cells = int(self.config.inflation_radius_m / self.config.grid_resolution_m)

        for obs_lat, obs_lon in obstacles:
            row, col = self._lat_lon_to_grid(obs_lat, obs_lon)
            if not self._in_bounds(row, col):
                continue

            # inflate obstacle by radius
            for dr in range(-inflation_cells, inflation_cells + 1):
                for dc in range(-inflation_cells, inflation_cells + 1):
                    if dr**2 + dc**2 <= inflation_cells**2:
                        nr, nc = row + dr, col + dc
                        if self._in_bounds(nr, nc):
                            self.grid[nr][nc] = True

    def plan(self, start_lat: float, start_lon: float,
             goal_lat: float, goal_lon: float) -> Optional[list[tuple[float, float]]]:
        """
        Runs A* from start to goal.
        Returns list of (lat, lon) waypoints or None if no path found.
        """
        start = self._lat_lon_to_grid(start_lat, start_lon)
        goal = self._grid_to_grid(*self._lat_lon_to_grid(goal_lat, goal_lon))

        if not self._in_bounds(*start) or not self._in_bounds(*goal):
            print("Start or goal out of bounds.")
            return None

        if self.grid[goal[0]][goal[1]]:
            print("Goal is inside an obstacle.")
            return None

        # A* implementation — research this and implement it yourself
        # hint: use heapq for the open set, a dict for g_scores,
        # and a dict to track parent of each node for path reconstruction
        pass

    def _heuristic(self, node: tuple, goal: tuple) -> float:
        """Euclidean distance heuristic in grid cells."""
        return euclidean_distance(node, goal)

    def _get_neighbors(self, row: int, col: int) -> list[tuple]:
        """Returns valid neighboring cells including diagonals."""
        neighbors = []
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = row + dr, col + dc
                if self._in_bounds(nr, nc) and not self.grid[nr][nc]:
                    neighbors.append((nr, nc))
        return neighbors

    def _diagonal_cost(self, current: tuple, neighbor: tuple) -> float:
        """Returns movement cost — diagonal moves cost more than cardinal."""
        dr = abs(current[0] - neighbor[0])
        dc = abs(current[1] - neighbor[1])
        if dr == 1 and dc == 1:
            return self.config.grid_resolution_m * 1.414  # sqrt(2)
        return self.config.grid_resolution_m

    def _lat_lon_to_grid(self, lat: float, lon: float) -> tuple[int, int]:
        """Convert lat/lon to grid cell (row, col)."""
        north_m, east_m = lat_lon_to_local(lat, lon, self.origin_lat, self.origin_lon)
        row = int(north_m / self.config.grid_resolution_m)
        col = int(east_m / self.config.grid_resolution_m)
        return row, col

    def _grid_to_lat_lon(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid cell back to lat/lon."""
        north_m = row * self.config.grid_resolution_m
        east_m = col * self.config.grid_resolution_m
        return local_to_lat_lon(north_m, east_m, self.origin_lat, self.origin_lon)

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def _grid_to_grid(self, row: int, col: int) -> tuple[int, int]:
        """Clamp grid coordinates to valid range."""
        return (
            max(0, min(row, self.rows - 1)),
            max(0, min(col, self.cols - 1))
        )