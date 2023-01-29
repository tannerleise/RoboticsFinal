import numpy as np
import matplotlib.pyplot as plt
import math
import random


###############################################################################
## Base Code
###############################################################################
class Node:
    """
    Node for RRT Algorithm.
    """
    def __init__(self, pt, parent=None):
        self.point = pt  # n-Dimensional point
        self.parent = parent  # Parent node
        self.path_from_parent = []  # List of points along the way from the parent node (for edge's collision checking)

def world_setup():
    '''
    Function that sets our c-space to the world we are currently using
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    m = np.load("../cspace.npy")
    state_bounds = np.array([[0, 900], [0, 900]])

    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        x = state[0]
        y = state[1]
        # print(state, x, y)
        if m[x][y] == 1:
            # print("m = ", m[x][y])
            # print("point", x, y, " In obstacle or out of bounds!!")
            return False
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]:
                print("The problem is the x")
                return False
            if state[dim] >= state_bounds[dim][1]:
                print("The problem is the y")
                return False
        return True

    return state_bounds, state_is_valid

def visualize_2D_graph(nodes, goal_point=None, filename=None):
    '''
    :param nodes: List of vertex locations
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param filename: Complete path to the file on which this plot will be saved
    :return: None
    '''
    goal_point = world_to_pixel(goal_point[0], goal_point[1])
    fig = plt.figure()
    m = np.load("../cspace.npy")

    r = []
    s = []

    for i in range(0,len(m[0])):
        for j in range(0,len(m[1])):
            if m[i][j] == 1:
                r.append(i)
                s.append(j)
    plt.scatter(r,s)


    goal_node = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:, 0], node_path[:, 1], '-b')
        # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal_node = node
            plt.plot(node.point[0], node.point[1], 'k^')
        else:
            plt.plot(node.point[0], node.point[1], 'ro')

    plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko')

    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                plt.plot(node_path[:, 0], node_path[:, 1], '-y')
                # plt.plot(cur_node.point[0], cur_node.point[1], "yellow")
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        plt.plot(goal_point[0], goal_point[1], 'gx')

    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()


def get_random_valid_vertex(state_is_valid, bounds):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None:  # Get starting vertex
        pt = np.random.rand(bounds.shape[0]) * (bounds[:, 1] - bounds[:, 0]) + bounds[:, 0]
        pt = pt.astype(int)
        if state_is_valid(pt):
            vertex = pt
    return vertex

def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: n-dimensional array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''
    distance_array = []
    # TODO: Your Code Here
    for node in node_list:
        distance = math.dist(node.point, q_point)
        distance_array.append(distance)

    node_idx = distance_array.index(min(distance_array))

    return node_list[node_idx]


def steer(from_point, to_point, delta_q):
    '''
    :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    dist = math.dist(from_point, to_point)
    if dist > delta_q:
        to_point_dimension = []
        c = dist / delta_q
        for i in range(len(to_point)):
            to_point_dimension.append(((to_point[i] - from_point[i]) / c) + from_point[i])
        to_point = to_point_dimension

    path = np.linspace(from_point, to_point, 10)
    path = path.astype(int)
    return path


def check_path_valid(path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''
    for i in path:
        valid = state_is_valid(i)
        if not valid:
            return False
    return True


def rrt(state_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
    '''
    RRT algorithm.
    If goal_point is set, your implementation should return once a path to the goal has been found
    (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations.
    If goal_point is None, it should build a graph without a goal and terminate after k iterations.

    :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
    :param starting_point: Point within state_bounds to grow the RRT from
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param k: Number of points to sample
    :param delta_q: Maximum distance allowed between vertices
    :returns List of RRT graph nodes
    '''
    starting_point = world_to_pixel(starting_point[0], starting_point[1])
    goal_point = world_to_pixel(goal_point[0], goal_point[1])
    node_list = []
    node_list.append(Node(starting_point, parent=None))  # Add Node at starting point with no parent
    if not state_is_valid(starting_point):
        print("Starting point is in a wall")
        return node_list

    if not state_is_valid(goal_point):
        print("Goal point is in a wall")
        return node_list

    for i in range(1, k):
        q_rand = get_random_valid_vertex(state_is_valid, state_bounds)

        # This is the random point that will occasionally point in the direction of the goal point
        if (goal_point is not None) and (random.random() < .10):
            q_rand = goal_point

        q_near = get_nearest_vertex(node_list, q_rand)
        q_new = steer(q_near.point, q_rand, delta_q)

        if check_path_valid(q_new, state_is_valid):
            node = Node(q_new[-1], parent=q_near)
            node.path_from_parent = q_new
            node_list.append(node)

            if (goal_point is not None) and math.dist(node.point, goal_point) < 5:
                # print("found it!!")
                return node_list

    return node_list


def get_path(nodes, goal_point):
    goal_point = world_to_pixel(goal_point[0], goal_point[1])
    path = []
    goal_node = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
        # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) < 5:
            goal_node = node

    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                cur_node = cur_node.parent
                world_node = pixel_to_world(cur_node.point[0], cur_node.point[1])
                path.append(world_node)
            else:
                break
    path.reverse()
    return path

# def world_to_pixel(x,y):
#     pixel_x = 450 + int(x * 30)
#     pixel_y = 240 - int(y * 30)
#     # I return y, x because in the file this is how they're drawn thanks to how I mapped it. Not an issue as long as we properly translate between them
#     return (pixel_y, pixel_x)

def pixel_to_world(x,y):
    world_x = x/30 - 15
    world_y = y/30 - 8
    # I return y, x because in the file this is how they're drawn thanks to how I mapped it. Not an issue as long as we properly translate between them
    return (world_x, world_y)


def world_to_pixel(x,y):
    pixel_x = 450 + int(x * 30)
    pixel_y = 240 - int(-y * 30)
    return (pixel_x, pixel_y)


if __name__ == "__main__":
    K = 7000  # Feel free to adjust as desired
    bounds, valid = world_setup()
    pt = get_random_valid_vertex(valid, bounds)


    start_point = (-5, 0)
    end_point = (10, 2)
    # end_point = (600,70)


    sheit = rrt(bounds, valid, start_point, end_point, K, 30)

    visualize_2D_graph(sheit, end_point, 'output.png')

    print(get_path(sheit, end_point))

