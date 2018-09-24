import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from queue import PriorityQueue
from bresenham import bresenham


def get_lat_lon(colliders):
    with open(colliders) as f:
        line0 = f.readline()
        lat0_lon0 = line0.replace('lat0 ', '').replace('lon0 ', '')
        lat0, lon0 = [float(x) for x in lat0_lon0.split(',')]

    return lat0, lon0


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    # return grid
    return grid, int(north_min), int(east_min)


straight_cost = 1
# diagonal_cost = 2**0.5
diagonal_cost = np.sqrt(2)
h_divider = 5


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    UP = (-1, 0, straight_cost)
    DOWN = (1, 0, straight_cost)
    LEFT = (0, -1, straight_cost)
    RIGHT = (0, 1, straight_cost)
    
    UP_LEFT = (-1, -1, diagonal_cost)
    UP_RIGHT = (-1, 1, diagonal_cost)
    DOWN_LEFT = (1, -1, diagonal_cost)
    DOWN_RIGHT = (1, 1, diagonal_cost)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node


    if x - 1 < 0 or grid[x - 1, y] >= 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] >= 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] >= 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] >= 1:
        valid.remove(Action.RIGHT)
 

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] >= 1:
        valid.remove(Action.UP_LEFT)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] >= 1:
        valid.remove(Action.UP_RIGHT)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] >= 1:
        valid.remove(Action.DOWN_LEFT)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] >= 1:
        valid.remove(Action.DOWN_RIGHT)
        
    return valid


def calc_additional_cost(grid, next_node):
    """
        check if the next node is adjacent to 1 or higher
    """
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = next_node
    
    additional_cost = 0
    
    if not x - 1 < 0:
        up_cost = grid[x-1, y]
        additional_cost += up_cost
    if not x + 1 > n:
        down_cost = grid[x+1, y]
        additional_cost += down_cost
    if not y - 1 < 0:
        left_cost = grid[x, y-1]
        additional_cost += left_cost
    if not y + 1 > m:
        right_cost = grid[x, y+1]
        additional_cost += right_cost
    
    if (not x - 1 < 0) and (not y - 1 < 0):
        up_left_cost = grid[x-1, y-1]
        additional_cost += up_left_cost
    if (not x - 1 < 0) and (not y + 1 > m):
        up_right_cost = grid[x-1, y+1]
        additional_cost += up_right_cost
    if (not x + 1 > n) and (not y - 1 < 0):
        down_left_cost = grid[x+1, y-1]
        additional_cost += down_left_cost
    if (not x + 1 > n) and (not y + 1 > m):
        down_right_cost = grid[x+1, y+1]
        additional_cost += down_right_cost

    return additional_cost


def heuristic(position, goal_position, factor=1):
    # return np.linalg.norm(np.array(position) - np.array(goal_position))
    # return np.abs(position[0] - goal_position[0]) + np.abs(position[1] - goal_position[1]) / factor
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                
                additional_cost = calc_additional_cost(grid, next_node)
                
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal, h_divider)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    
                    branch_cost += additional_cost
                    queue_cost += additional_cost
                    
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def plot_path_on_grid(grid, path, start, goal, flip=True, scatter=True):
    if flip:
        plt.imshow(grid, cmap='Greys', origin='lower')
    else:
        plt.imshow(grid, cmap='Greys')

    plt.plot(start[1], start[0], 'x')
    plt.plot(goal[1], goal[0], 'x')

    if path is not None:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
        if scatter:
            plt.scatter(pp[:, 1], pp[:, 0])

    if flip:
        plt.xlabel('EAST')
        plt.ylabel('NORTH')

    plt.show()


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):
    if path is not None:
        pruned_path = [p for p in path]

        # prune the path!
        i = 0
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2])

            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if collinearity_check(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
    else:
        pruned_path = path

    return pruned_path


def clear_path(grid, p1, p2):
    points = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))

    for x, y in points:
        if grid[x][y] >= 1:
            return False

    return True


def bresenham_raytracing(grid, path):
    if len(path) <= 3:
        # no pruning needed
        return path

    elif clear_path(grid, path[0], path[-1]):

        # return first and last waypoint
        return [path[0], path[-1]]

    else:
        # split path in two segments 
        path_div = len(path) // 2

        # prune both segements
        head = bresenham_raytracing(grid, path[:path_div])
        tail = bresenham_raytracing(grid, path[max(0, path_div - 1):])

        # return combined path
        return head + tail[1:]


def optimize_path(grid, path):
    print('prune, then bresenham')
    pruned_path = prune_path(path)
    print(len(pruned_path))
    prune_bresenham = bresenham_raytracing(grid, pruned_path)
    print(len(prune_bresenham))

    print('bresenham, then prune')
    bresenham_path = bresenham_raytracing(grid, path)
    print(len(bresenham_path))
    bresenham_prune = prune_path(bresenham_path)
    print(len(bresenham_prune))

    print('using shortest path')

    path = min([prune_bresenham, bresenham_prune], key=len)

    return path
