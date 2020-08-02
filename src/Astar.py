import src.AllConstants as Const
from src.Utility_Func import jpg_to_bin_image
import matplotlib.pyplot as plt
import numpy as np
import math
from src.Utility_Func import obstacle_scale
from src.Utility_Func import calc_path_length
from src.Utility_Func import grid_square


class Node(object):
    def __init__(self, value, point, edge_length=1, descr='Node'):
        self.value = value
        self.point = point
        self.edgeLength = edge_length
        self.descr = descr
        self.parent = None
        self.H = 0
        self.G = 0

    def __str__(self):
        return str(self.point) + "," + str(self.G)

    def __cmp__(self, other):
        return self.priority - other.priority

    def value_calc(self, grid_raw):
        x, y = self.point
        e = self.edgeLength
        self.value = np.sum(grid_raw[x * e: (x + 1) * e, y * e:(y + 1) * e])


def children(node, grid):
    node_x, node_y = node.point
    links = []
    for x in range(3):
        for y in range(3):
            try:
                if grid[node_x + x - 1, node_y + y - 1].value == 0:
                    links.append(grid[node_x + x - 1, node_y + y - 1])
            except:
                continue
    return links


def h1func(node1, node2):
    return abs(node1.point[0] - node2.point[0]) + abs(node1.point[1] - node2.point[1])


def h2func(node1, node2):
    return math.sqrt((node1.point[0] - node2.point[0]) ** 2 + (node1.point[1] - node2.point[1]) ** 2)


def a_star(start, goal, grid):
    # The open and closed sets
    openset = set()
    closedset = set()
    # Current point is the starting point
    current = start
    # Add the starting point to the open set
    openset.add(current)
    # While the open set is not empty
    while openset:
        # Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o: o.G + o.H)
        # If it is the item we want, retrace the path and return it
        if current.descr == 'goal':
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            print('Path is found')
            return path[::-1]
        # Remove the item from the open set
        openset.remove(current)
        # Add it to the closed set
        closedset.add(current)
        # Loop through the node's children/siblings
        for node in children(current, grid):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue
            # Otherwise if it is already in the open set
            if node in openset:
                # Check if we beat the G score
                new_g = current.G + 1
                new_h = h2func(node, goal)
                if node.G + node.H > new_g + new_h:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    node.H = new_h
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + 1
                node.H = h2func(node, goal)
                # Set the parent to our current item
                node.parent = current
                # Add it to the set
                openset.add(node)
    # Throw an exception if there is no path
    raise ValueError('No Path Found')


def run_astar(grid_raw, start_cord, goal_cord, resolution=1):
    edge_size = int(len(grid_raw) / resolution)
    # emptyNode = Node(0, (0, 0), resolution)
    grid_node = np.empty((edge_size, edge_size), Node)

    start = Node(0, (int(start_cord[0] / resolution), int(start_cord[1] / resolution)), resolution, 'start')
    start.value_calc(grid_raw)
    goal = Node(0, (int(goal_cord[0] / resolution), int(goal_cord[1] / resolution)), resolution, 'goal')
    goal.value_calc(grid_raw)
    if start.value != 0 or goal.value != 0:
        raise Exception('Goal or start are in occupied cells')

    for x in range(edge_size):
        for y in range(edge_size):
            grid_node[x, y] = Node(0, (x, y), resolution)
            grid_node[x, y].value_calc(grid_raw)

    grid_node[start.point[0], start.point[1]] = start
    grid_node[goal.point[0], goal.point[1]] = goal
    return a_star(start, goal, grid_node)


def perform_dijkstra(floor_img_adr=Const.FLOOR_IMG_ADR, start_state=Const.start_state, goal_state=Const.goal_state,
                     is_print=False):
    start_cord = start_state[0]
    goal_cord = goal_state[0]

    binary_grid = jpg_to_bin_image(floor_img_adr=floor_img_adr)
    import_grid = np.around(binary_grid * 1.01, decimals=0)
    r_radios = Const.ROBOT_RADIUS
    # plt.imshow(import_grid, cmap='hot', interpolation='nearest')

    obstacle_grid = obstacle_scale(import_grid, r_radios)
    import_grid = np.copy(obstacle_grid)
    square_grid = grid_square(obstacle_grid)

    if is_print:
        plt.imshow(obstacle_grid, cmap='hot', interpolation='nearest')
        plt.show()
        # plt.imshow(square_grid, cmap='hot', interpolation='nearest')
        # plt.show()

    path = list()
    for p in range(int(math.log(len(square_grid), 2)), 0, -1):
        try:
            path = run_astar(square_grid, start_cord, goal_cord, 2 ** p)
            break
        except Exception as e:
            print(e)

    path_no_scale = list()
    path_no_scale.append(start_cord)
    for node in path:
        x, y = node.point
        res = node.edgeLength
        path_no_scale.append((x * res, y * res))
        import_grid[x * res: (x + 1) * res, y * res: (y + 1) * res] = 0.4
    import_grid[start_cord[0] - 5:start_cord[0] + 6, start_cord[1] - 5:start_cord[1] + 6] = 0.6
    import_grid[goal_cord[0] - 5:goal_cord[0] + 6, goal_cord[1] - 5:goal_cord[1] + 6] = 0.2
    path_no_scale.append(goal_cord)

    plt.imshow(import_grid[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
    plt.show()

    calc_path_length(path_no_scale)
    print("END A*")


if __name__ == '__main__':
    perform_dijkstra(floor_img_adr=Const.FLOOR_IMG_ADR, start_state=Const.start_state, goal_state=Const.goal_state,
                     is_print=True)
