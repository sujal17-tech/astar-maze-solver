"""
Maze Solver using A* Search
============================
Internship Project 1

Features:
- Represents a maze as a 2D grid (0 = free cell, 1 = wall)
- Models each cell as a "node" with position, cost, and parent link
- Implements A* search using Manhattan or Euclidean heuristic
- Returns the shortest path from start to goal
- Handles the case where the goal is unreachable
- Visualizes the explored nodes + final path using matplotlib
  (falls back to a console/text visualization if matplotlib is unavailable)

Author: Sujal Bhedurkar 
"""

import heapq
import math
import time


# ---------------------------------------------------------------------------
# 1. NODE REPRESENTATION
# ---------------------------------------------------------------------------
class Node:
    """
    Represents a single cell in the maze grid during the A* search.

    Attributes:
        position : (row, col) tuple -> location of this node in the grid
        parent   : Node -> the node we came from (used to reconstruct path)
        g        : cost from the start node to this node (actual distance so far)
        h        : heuristic estimate of cost from this node to the goal
        f        : g + h  (total estimated cost of path through this node)
    """

    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    # heapq needs a way to compare Nodes when f values are equal
    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)


# ---------------------------------------------------------------------------
# 2. MAZE REPRESENTATION
# ---------------------------------------------------------------------------
class Maze:
    """
    Wraps a 2D grid and exposes helper methods needed by the search:
    - checking bounds
    - checking walls
    - getting walkable neighbors
    """

    def __init__(self, grid, start, goal):
        """
        grid  : list[list[int]]  -> 0 = free cell, 1 = wall
        start : (row, col)
        goal  : (row, col)
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.start = start
        self.goal = goal

        self._validate()

    def _validate(self):
        if not self.in_bounds(self.start):
            raise ValueError(f"Start {self.start} is outside the grid.")
        if not self.in_bounds(self.goal):
            raise ValueError(f"Goal {self.goal} is outside the grid.")
        if self.is_wall(self.start):
            raise ValueError(f"Start {self.start} is on a wall cell.")
        if self.is_wall(self.goal):
            raise ValueError(f"Goal {self.goal} is on a wall cell.")

    def in_bounds(self, pos):
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_wall(self, pos):
        r, c = pos
        return self.grid[r][c] == 1

    def neighbors(self, pos, allow_diagonal=False):
        """
        Returns walkable neighbor positions of `pos`.
        By default only 4-directional movement (up/down/left/right).
        Set allow_diagonal=True for 8-directional movement.
        """
        r, c = pos
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
        if allow_diagonal:
            moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        result = []
        for dr, dc in moves:
            new_pos = (r + dr, c + dc)
            if self.in_bounds(new_pos) and not self.is_wall(new_pos):
                result.append(new_pos)
        return result


# ---------------------------------------------------------------------------
# 3. HEURISTIC FUNCTIONS
# ---------------------------------------------------------------------------
def manhattan_distance(a, b):
    """|x1-x2| + |y1-y2|  -- best for 4-directional grids."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean_distance(a, b):
    """straight-line distance -- useful for 8-directional / free movement."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ---------------------------------------------------------------------------
# 4. A* SEARCH ALGORITHM
# ---------------------------------------------------------------------------
def a_star(maze, heuristic="manhattan", allow_diagonal=False):
    """
    Runs A* search on the given Maze object.

    Returns a dict with:
        path         : list of (row, col) from start to goal (or None if unreachable)
        cost         : total path cost (or None)
        explored     : list of positions expanded, in order (for visualization)
        explored_set : set of all explored positions
    """
    heuristic_fn = manhattan_distance if heuristic == "manhattan" else euclidean_distance

    start_node = Node(maze.start, parent=None, g=0,
                       h=heuristic_fn(maze.start, maze.goal))

    open_heap = []          # priority queue (min-heap) ordered by f value
    heapq.heappush(open_heap, start_node)

    # best known g-cost for each position (lets us skip worse duplicate paths)
    best_g = {maze.start: 0}

    # keep track of nodes currently "in" the open heap so we can avoid
    # re-adding stale/duplicate entries unnecessarily
    open_set = {maze.start}

    closed_set = set()      # fully expanded (finalized) nodes
    explored_order = []     # order in which nodes were expanded (for viz)

    while open_heap:
        current = heapq.heappop(open_heap)
        open_set.discard(current.position)

        # Skip if we've already found a better path to this node
        if current.position in closed_set:
            continue

        closed_set.add(current.position)
        explored_order.append(current.position)

        # ---- GOAL CHECK ----
        if current.position == maze.goal:
            path = _reconstruct_path(current)
            return {
                "path": path,
                "cost": current.g,
                "explored": explored_order,
                "explored_set": closed_set,
            }

        # ---- EXPAND NEIGHBORS ----
        for neighbor_pos in maze.neighbors(current.position, allow_diagonal):
            if neighbor_pos in closed_set:
                continue

            # cost of moving to neighbor (diagonal moves cost slightly more)
            step_cost = (
                math.sqrt(2)
                if allow_diagonal and neighbor_pos[0] != current.position[0]
                and neighbor_pos[1] != current.position[1]
                else 1
            )
            tentative_g = current.g + step_cost

            if neighbor_pos not in best_g or tentative_g < best_g[neighbor_pos]:
                best_g[neighbor_pos] = tentative_g
                h = heuristic_fn(neighbor_pos, maze.goal)
                neighbor_node = Node(neighbor_pos, parent=current, g=tentative_g, h=h)
                heapq.heappush(open_heap, neighbor_node)
                open_set.add(neighbor_pos)

    # ---- OPEN LIST EMPTY -> GOAL UNREACHABLE ----
    return {
        "path": None,
        "cost": None,
        "explored": explored_order,
        "explored_set": closed_set,
    }


def _reconstruct_path(node):
    """Walks parent pointers from goal back to start, then reverses."""
    path = []
    while node is not None:
        path.append(node.position)
        node = node.parent
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# 5. CONSOLE (TEXT) VISUALIZATION -- always works, no dependencies
# ---------------------------------------------------------------------------
def print_maze(maze, path=None, explored=None):
    """
    Prints the maze to the console.
    Legend:
        #  wall
        .  free cell
        S  start
        G  goal
        *  final path
        o  explored (but not on final path)
    """
    path_set = set(path) if path else set()
    explored_set = set(explored) if explored else set()

    print()
    for r in range(maze.rows):
        row_str = ""
        for c in range(maze.cols):
            pos = (r, c)
            if pos == maze.start:
                row_str += "S "
            elif pos == maze.goal:
                row_str += "G "
            elif maze.grid[r][c] == 1:
                row_str += "# "
            elif pos in path_set:
                row_str += "* "
            elif pos in explored_set:
                row_str += "o "
            else:
                row_str += ". "
        print(row_str)
    print()


# ---------------------------------------------------------------------------
# 6. MATPLOTLIB VISUALIZATION -- nicer, optional
# ---------------------------------------------------------------------------
def plot_maze(maze, path=None, explored=None, title="A* Maze Solver",
              save_path=None, show=True):
    """
    Draws the maze using matplotlib:
        - black cells  = walls
        - white cells  = free space
        - light blue   = explored cells
        - gold         = final path
        - green circle = start
        - red circle   = goal
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("[!] matplotlib not installed -- falling back to console view.")
        print_maze(maze, path, explored)
        return

    fig, ax = plt.subplots(figsize=(maze.cols / 2, maze.rows / 2))

    # Draw grid cells
    for r in range(maze.rows):
        for c in range(maze.cols):
            color = "black" if maze.grid[r][c] == 1 else "white"
            ax.add_patch(patches.Rectangle((c, maze.rows - r - 1), 1, 1,
                                            facecolor=color, edgecolor="gray", linewidth=0.5))

    # Explored cells
    if explored:
        for (r, c) in explored:
            if (r, c) not in (maze.start, maze.goal):
                ax.add_patch(patches.Rectangle((c, maze.rows - r - 1), 1, 1,
                                                facecolor="#AED6F1", edgecolor="gray", linewidth=0.5))

    # Final path
    if path:
        for (r, c) in path:
            if (r, c) not in (maze.start, maze.goal):
                ax.add_patch(patches.Rectangle((c, maze.rows - r - 1), 1, 1,
                                                facecolor="gold", edgecolor="gray", linewidth=0.5))

    # Start / goal markers
    sr, sc = maze.start
    gr, gc = maze.goal
    ax.add_patch(patches.Circle((sc + 0.5, maze.rows - sr - 1 + 0.5), 0.3, facecolor="green"))
    ax.add_patch(patches.Circle((gc + 0.5, maze.rows - gr - 1 + 0.5), 0.3, facecolor="red"))

    ax.set_xlim(0, maze.cols)
    ax.set_ylim(0, maze.rows)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title)

    if save_path:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        print(f"[+] Plot saved to: {save_path}")
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# 7. DEMO / MAIN
# ---------------------------------------------------------------------------
def build_sample_maze():
    """
    0 = free, 1 = wall
    Returns (grid, start, goal)
    """
    grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 1, 0],
        [0, 1, 1, 1, 0, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 1, 1, 0, 1, 0],
        [0, 1, 0, 1, 1, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    ]
    start = (0, 0)
    goal = (9, 9)
    return grid, start, goal


def build_unreachable_maze():
    """A maze where the goal is completely walled off (to test failure case)."""
    grid = [
        [0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
    ]
    start = (0, 0)
    goal = (0, 4)   # sealed off by the wall column at index 3
    return grid, start, goal


def run_demo():
    print("=" * 60)
    print("A* MAZE SOLVER DEMO")
    print("=" * 60)

    # ---- Case 1: solvable maze ----
    grid, start, goal = build_sample_maze()
    maze = Maze(grid, start, goal)

    t0 = time.time()
    result = a_star(maze, heuristic="manhattan", allow_diagonal=False)
    elapsed = time.time() - t0

    print("\n[Case 1] Solvable maze")
    print(f"Start: {start}  Goal: {goal}")
    if result["path"]:
        print(f"Path found! Length = {len(result['path'])} steps, "
              f"cost = {result['cost']}, time = {elapsed:.4f}s")
        print(f"Nodes explored: {len(result['explored'])}")
        print(f"Path: {result['path']}")
    else:
        print("No path found (unexpected for this maze!)")

    print_maze(maze, path=result["path"], explored=result["explored"])
    plot_maze(maze, path=result["path"], explored=result["explored"],
               title="A* Search - Solvable Maze",
               save_path="solvable_maze_result.png", show=False)

    # ---- Case 2: unreachable goal ----
    grid2, start2, goal2 = build_unreachable_maze()
    maze2 = Maze(grid2, start2, goal2)
    result2 = a_star(maze2, heuristic="manhattan")

    print("\n[Case 2] Unreachable goal")
    print(f"Start: {start2}  Goal: {goal2}")
    if result2["path"] is None:
        print("Correctly detected: NO PATH EXISTS (goal unreachable).")
    else:
        print("Unexpected: a path was found.")

    print_maze(maze2, path=result2["path"], explored=result2["explored"])
    plot_maze(maze2, path=result2["path"], explored=result2["explored"],
               title="A* Search - Unreachable Goal",
               save_path="unreachable_maze_result.png", show=False)

    print("\nDone. Check 'solvable_maze_result.png' and "
          "'unreachable_maze_result.png' for saved plots.")


if __name__ == "__main__":
    run_demo()
