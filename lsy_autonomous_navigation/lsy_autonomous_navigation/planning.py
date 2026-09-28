"""ROS-independent grid planning utilities, kept testable outside ROS."""
from __future__ import annotations
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Cell = Tuple[int, int]

MOVES = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)

def in_bounds(cell: Cell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height

def index(cell: Cell, width: int) -> int:
    x, y = cell
    return y * width + x

def is_free(cell: Cell, grid: Sequence[int], width: int, height: int,
            occupied_threshold: int = 50, allow_unknown: bool = False) -> bool:
    if not in_bounds(cell, width, height):
        return False
    value = grid[index(cell, width)]
    if value < 0:
        return allow_unknown
    return value < occupied_threshold

def heuristic(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def neighbors(cell: Cell, grid: Sequence[int], width: int, height: int) -> Iterable[Tuple[Cell, float]]:
    x, y = cell
    for dx, dy, cost in MOVES:
        nxt = (x + dx, y + dy)
        if not is_free(nxt, grid, width, height):
            continue
        # Forbid diagonal corner-cutting through obstacles.
        if dx != 0 and dy != 0:
            if not is_free((x + dx, y), grid, width, height):
                continue
            if not is_free((x, y + dy), grid, width, height):
                continue
        yield nxt, cost

def reconstruct(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar(grid: Sequence[int], width: int, height: int, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if not is_free(start, grid, width, height) or not is_free(goal, grid, width, height):
        return None
    frontier = [(heuristic(start, goal), 0.0, start)]
    came_from: Dict[Cell, Cell] = {}
    best_g: Dict[Cell, float] = {start: 0.0}
    closed = set()
    while frontier:
        _, g, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal:
            return reconstruct(came_from, current)
        closed.add(current)
        for nxt, step_cost in neighbors(current, grid, width, height):
            new_g = g + step_cost
            if new_g < best_g.get(nxt, math.inf):
                best_g[nxt] = new_g
                came_from[nxt] = current
                heapq.heappush(frontier, (new_g + heuristic(nxt, goal), new_g, nxt))
    return None
