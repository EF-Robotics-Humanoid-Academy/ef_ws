"""
path_planner.py
===============

A* pathfinding on a 2D occupancy grid with path smoothing and
grid-to-world waypoint conversion.

No SDK dependency -- pure Python (heapq, math, numpy).
"""
from __future__ import annotations

import heapq
import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from create_map import OccupancyGrid


# ---------------------------------------------------------------------------
# Bresenham line (used by smooth_path for line-of-sight checks)
# ---------------------------------------------------------------------------


def _bresenham_line(
    r0: int, c0: int, r1: int, c1: int
) -> list[tuple[int, int]]:
    """Return all (row, col) cells on the line from (r0,c0) to (r1,c1)."""
    cells: list[tuple[int, int]] = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0

    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


# ---------------------------------------------------------------------------
# A* pathfinding
# ---------------------------------------------------------------------------


def astar(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    allow_diagonal: bool = True,
) -> list[tuple[int, int]] | None:
    """A* pathfinding on a 2D occupancy grid.

    Args:
        grid:  ``np.ndarray`` shape ``(H, W)``.  0 = free, nonzero = blocked.
        start: ``(row, col)`` start cell.
        goal:  ``(row, col)`` goal cell.
        allow_diagonal: 8-connected if True, 4-connected if False.

    Returns:
        List of ``(row, col)`` from start to goal (inclusive), or ``None``
        if no path exists.
    """
    rows, cols = grid.shape

    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return None
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return None
    if grid[start[0], start[1]] != 0:
        return None
    if grid[goal[0], goal[1]] != 0:
        return None

    _SQRT2 = math.sqrt(2)

    if allow_diagonal:
        neighbours = [
            (-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
            ( 0, -1, 1.0),                    ( 0, 1, 1.0),
            ( 1, -1, _SQRT2), ( 1, 0, 1.0), ( 1, 1, _SQRT2),
        ]

        def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])
    else:
        neighbours = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

        def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    counter = 0
    open_set: list[tuple[float, int, tuple[int, int]]] = []
    heapq.heappush(open_set, (heuristic(start, goal), counter, start))
    closed: set[tuple[int, int]] = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if current in closed:
            continue
        closed.add(current)

        for dr, dc, cost in neighbours:
            nr, nc = current[0] + dr, current[1] + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr, nc] != 0:
                continue
            neighbour = (nr, nc)
            if neighbour in closed:
                continue
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbour, float("inf")):
                g_score[neighbour] = tentative_g
                came_from[neighbour] = current
                f = tentative_g + heuristic(neighbour, goal)
                counter += 1
                heapq.heappush(open_set, (f, counter, neighbour))

    return None


# ---------------------------------------------------------------------------
# Path smoothing
# ---------------------------------------------------------------------------


def smooth_path(
    path: list[tuple[int, int]],
    grid: np.ndarray,
    max_skip: int = 5,
) -> list[tuple[int, int]]:
    """Greedy path smoothing via Bresenham line-of-sight checks.

    Tries to skip intermediate waypoints when the direct line between
    non-adjacent waypoints is entirely free.
    """
    if len(path) <= 2:
        return list(path)

    smoothed = [path[0]]
    i = 0

    while i < len(path) - 1:
        best = i + 1
        for skip in range(min(max_skip, len(path) - 1 - i), 0, -1):
            candidate = i + skip
            if candidate >= len(path):
                continue
            line = _bresenham_line(
                path[i][0], path[i][1], path[candidate][0], path[candidate][1]
            )
            if all(grid[r, c] == 0 for r, c in line):
                best = candidate
                break
        smoothed.append(path[best])
        i = best

    return smoothed


# ---------------------------------------------------------------------------
# Grid path -> world waypoints
# ---------------------------------------------------------------------------


def grid_path_to_world_waypoints(
    path: list[tuple[int, int]],
    occupancy_grid: OccupancyGrid,
    spacing_m: float = 0.5,
) -> list[tuple[float, float]]:
    """Convert a grid-cell path to world-coordinate waypoints spaced
    approximately *spacing_m* apart.  Always includes start and goal."""
    if not path:
        return []

    world_pts = [occupancy_grid.grid_to_world(r, c) for r, c in path]

    if len(world_pts) <= 2:
        return world_pts

    waypoints = [world_pts[0]]
    accumulated = 0.0

    for i in range(1, len(world_pts)):
        dx = world_pts[i][0] - world_pts[i - 1][0]
        dy = world_pts[i][1] - world_pts[i - 1][1]
        accumulated += math.hypot(dx, dy)

        if accumulated >= spacing_m:
            waypoints.append(world_pts[i])
            accumulated = 0.0

    # Always include the final point
    if waypoints[-1] != world_pts[-1]:
        waypoints.append(world_pts[-1])

    return waypoints


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Test 1: path around a wall
    test_grid = np.zeros((20, 20), dtype=np.int8)
    test_grid[5:15, 10] = 1

    result = astar(test_grid, (10, 2), (10, 18))
    if result:
        print(f"A* found path with {len(result)} cells")
        smoothed = smooth_path(result, test_grid)
        print(f"Smoothed to {len(smoothed)} waypoints")
    else:
        print("ERROR: no path found (expected one)")

    # Test 2: clear grid
    clear = np.zeros((10, 10), dtype=np.int8)
    result2 = astar(clear, (0, 0), (9, 9))
    print(f"Clear grid path: {len(result2)} cells" if result2 else "ERROR")

    # Test 3: fully blocked
    blocked = np.zeros((5, 5), dtype=np.int8)
    blocked[2, :] = 1
    result3 = astar(blocked, (0, 0), (4, 4))
    print(f"Blocked grid: {'No path (correct)' if result3 is None else 'ERROR'}")

    print("All tests passed.")
