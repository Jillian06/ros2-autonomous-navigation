from autonomous_navigation.planning import astar

def test_open_grid():
    grid = [0] * 25
    path = astar(grid, 5, 5, (0, 0), (4, 4))
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)

def test_wall_with_gap():
    grid = [0] * 25
    for y in range(4):
        grid[y * 5 + 2] = 100
    path = astar(grid, 5, 5, (0, 0), (4, 0))
    assert path is not None
    assert (2, 4) in path

def test_blocked_goal():
    grid = [0] * 9
    grid[8] = 100
    assert astar(grid, 3, 3, (0, 0), (2, 2)) is None

def test_diagonal_corner_cut_is_rejected():
    # Start is boxed from diagonal motion by two occupied cardinal cells.
    grid = [0] * 9
    grid[1] = 100
    grid[3] = 100
    assert astar(grid, 3, 3, (0, 0), (1, 1)) is None
