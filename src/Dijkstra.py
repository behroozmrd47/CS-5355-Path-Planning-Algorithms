import src.AllConstants as Const
import matplotlib.pyplot as plt
import numpy as np
from ordered_set import OrderedSet
import src.Astar as A_star
from src.Utility_Func import dist


class Node(object):
    def __init__(self, cost, value, point, descr='Node'):
        self.G = cost
        self.value = value
        self.point = point
        self.descr = descr
        self.parent = None

    def __str__(self):
        return str(self.point) + "," + str(self.G)

    def value_calc(self, gridraw):
        x, y = self.point
        self.value = gridraw[x, y]


def children(node, grid):
    nodex, nodey = node.point
    links = []
    for x in range(3):
        for y in range(3):
            try:
                if grid[nodex + x - 1, nodey + y - 1].value == 0:
                    links.append(grid[nodex + x - 1, nodey + y - 1])
            except:
                continue
    return links


def dijkstra(start, grid):
    # The open and closed sets
    openset = OrderedSet()
    closedset = OrderedSet()
    # Current point is the starting point
    current = start
    # Add the starting point to the open set
    openset.add(current)
    # While the open set is not empty
    while openset:
        # Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o: o.G)
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
                new_g = current.G + dist(node.point, current.point)
                # new_h = 0
                if node.G > new_g:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    # node.H = new_h
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + dist(node.point, current.point)
                # node.H = 0
                # Set the parent to our current item
                node.parent = current
                # Add it to the set
                openset.add(node)
    # Throw an exception if there is no path
    raise ValueError('No Path Found')


def run_dijkstra(grid_raw, start_cord, goal_cord):
    grid_node = np.empty((len(grid_raw), len(grid_raw)), Node)

    start = Node(0, 0, start_cord, 'start')
    start.value_calc(grid_raw)
    goal = Node(0, 0, goal_cord, 'goal')
    goal.value_calc(grid_raw)
    if start.value != 0 or goal.value != 0:
        raise Exception('Goal or start are in occupied cells')

    for x in range(len(grid_node)):
        for y in range(len(grid_node[0])):
            grid_node[x, y] = Node(0, 0, (x, y))
            grid_node[x, y].value_calc(grid_raw)

    grid_node[start.point[0], start.point[1]] = start
    grid_node[goal.point[0], goal.point[1]] = goal
    return dijkstra(start, grid_node)


def perform_dijkstra(grid_raw, start_state=Const.start_state, goal_state=Const.goal_state):
    start_cord = start_state[0]
    goal_cord = goal_state[0]

    importGrid = np.copy(A_star.obsScaleGrid)
    rRadios = Const.ROBOT_RADIUS
    plt.imshow(importGrid, cmap='hot', interpolation='nearest')
    obsScaleGrid = np.copy(A_star.obsScaleGrid)
    plt.imshow(obsScaleGrid, cmap='hot', interpolation='nearest')
    importGrid = np.copy(obsScaleGrid)

    path = run_dijkstra(obsScaleGrid, start_cord, goal_cord)

    for node in path:
        x, y = node.point
        importGrid[x - 3:x + 4, y - 3:y + 4] = 0.4
    importGrid[start_cord[0] - 5:start_cord[0] + 6, start_cord[1] - 5:start_cord[1] + 6] = 0.6
    importGrid[goal_cord[0] - 5:goal_cord[0] + 6, goal_cord[1] - 5:goal_cord[1] + 6] = 0.2

    plt.imshow(importGrid[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
    plt.show()

    total = 0
    for i in range(len(path) - 1):
        total = total + dist(path[i].point, path[i + 1].point)
    print
    total

    print("END Dijkstra")
