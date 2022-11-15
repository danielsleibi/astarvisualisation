# A* visualization program

import numpy as np
import sys

FLOAT_MAX = sys.float_info.max


class Cell:
    def __init__(self, parent, f_value, g_value, h_value):
        self._parent = parent
        self._f_value = f_value
        self._g_value = g_value
        self._h_value = h_value

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, v):
        self._parent = v

    @property
    def f_value(self):
        return self._f_value

    @f_value.setter
    def f_value(self, v):
        self._f_value = v

    @property
    def g_value(self):
        return self._g_value

    @g_value.setter
    def g_value(self, v):
        self._g_value = v

    @property
    def h_value(self):
        return self._h_value

    @h_value.setter
    def h_value(self, v):
        self._h_value = v


class AStarUtils:
    """Utility functions"""

    def __init__(self, width, height, grid):
        self._width = width
        self._height = height
        self._grid = grid

    @staticmethod
    def is_goal(x, y, gx, gy):
        """Checks if goal is reached"""
        if x == gx and y == gy:
            return True
        return False

    # Trace optimal path
    @staticmethod
    def trace_path(cells, goal):
        """Return the most optimal path found"""
        x = goal[0]
        y = goal[1]
        path = []

        while cells[x][y].parent != (x, y):
            path.append((x, y))
            parent = cells[x][y].parent
            x = parent[0]
            y = parent[1]
        path.append((x, y))
        return path

    # euclidean heuristic
    @staticmethod
    def heuristic(start, goal):
        """Calculates the heuristic value"""
        diff_x = (goal[0] - start[0])
        diff_y = (goal[1] - start[1])
        return np.sqrt((diff_x * diff_x) + (diff_y * diff_y))

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    # Check if cell is valid
    def is_valid(self, x, y):
        """Checks if cell is in bounds of the grid"""
        if (x < 0) or (x >= self._width) or (y < 0) or (y >= self._height):
            return False
        return True

    # Check if is blocked
    def is_blocked(self, x, y):
        """Checks if cell is a block or path"""
        if self._grid[x][y] == 1:
            return True
        return False

    def initialize_cell_data(self):
        """Initializes cell data"""
        cells = [[] for _ in range(self._width)]
        for x in range(self._width):
            for y in range(self._height):
                cells[x].append(Cell((-1, -1), FLOAT_MAX, FLOAT_MAX, FLOAT_MAX))
        return cells

    def process_cell(self, cells, open_list, closed_list, x, y, gx, gy, px, py):
        """Process cell for expanding the frontier (open list) and calculating evaluation values"""
        if not self.is_valid(x, y):
            return False
        cell = cells[x][y]
        if AStarUtils.is_goal(x, y, gx, gy):
            cell.parent = (px, py)
            return True
        if not closed_list[x][y] and not self.is_blocked(x, y):
            pcell = cells[px][py]
            g_new = pcell.g_value + 1.0
            h_new = self.heuristic((x, y), (gx, gy))
            f_new = g_new + h_new

            if cell.f_value == FLOAT_MAX or cell.f_value > f_new:
                open_list.append((f_new, (x, y)))
                cell.f_value = f_new
                cell.g_value = g_new
                cell.h_value = h_new
                cell.parent = (px, py)
        return False


def a_star(start, goal, utils):
    """Calculate optimal path using A* Algorithm"""
    # predefined conditions
    if not utils.is_valid(start[0], start[1]):
        raise ValueError("Invalid start cell")

    if not utils.is_valid(goal[0], goal[1]):
        raise ValueError("Invalid goal cell")

    if utils.is_blocked(start[0], start[1]):
        raise ValueError("Blocked start cell")

    if utils.is_blocked(goal[0], goal[1]):
        raise ValueError("Blocked goal cell")

    if AStarUtils.is_goal(start[0], start[1], goal[0], goal[1]):
        return []

    open_list = []
    closed_list = [[False for _ in range(utils.height)] for _ in range(utils.width)]
    cells = utils.initialize_cell_data()
    x, y = start[0], start[1]
    start_cell = cells[start[0]][start[1]]
    start_cell.f_value = 0.0
    start_cell.g_value = 0.0
    start_cell.h_value = 0.0
    start_cell.parent = (x, y)

    open_list.append((0.0, (x, y)))

    path = []
    while open_list:
        c = open_list[0]
        open_list.pop(0)
        x = c[1][0]
        y = c[1][1]
        closed_list[x][y] = True

        # left
        left_result = utils.process_cell(cells, open_list, closed_list, x - 1, y, goal[0], goal[1], x, y)
        # right
        right_result = utils.process_cell(cells, open_list, closed_list, x + 1, y, goal[0], goal[1], x, y)
        # up
        up_result = utils.process_cell(cells, open_list, closed_list, x, y + 1, goal[0], goal[1], x, y)
        # down
        down_result = utils.process_cell(cells, open_list, closed_list, x, y - 1, goal[0], goal[1], x, y)

        if left_result or right_result or up_result or down_result:
            path = utils.trace_path(cells, goal)
            break
    return [path, cells]
