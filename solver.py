import sys
import timeit

# from profilehooks import profile
import random

import math

from node import Node
from support.angle import Angle
from support.problem_spec import ProblemSpec
from support.robot_config import make_robot_config_from_ee1, write_robot_config_list_to_file
from tester import test_config_equality, test_obstacle_collision, test_self_collision, test_environment_bounds

"""
Example of code for performing BFS on a state graph, including a class representing a state graph node.

COMP3702 2019 Assignment 2 Support Code

Last updated by njc 17/09/19
"""
PRIMITIVE_STEP = 1e-3


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    # def __str__(self):
    #     return [str(round(a.in_degrees(), 8)) for a in self.ee1_angles]

    def get_successors(self):
        return self.neighbors

    @classmethod
    def convert_to_degrees(cls, PRIMITIVE_STEP):
        return PRIMITIVE_STEP * 180 / math.pi


# @profile
def solve(spec, output):
    """
    An example solve method containing code to perform a breadth first search of the state graph and return a list of
    configs which form a path through the state graph between the initial and the goal. Note that this path will not
    satisfy the primitive step requirement - you will need to interpolate between the configs in the returned list.

    If you wish to use this code, you may either copy this code into your own file or add your existing code to this
    file.

    :param spec: ProblemSpec object
    :return: List of configs forming a path through the graph from initial to goal
    """

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    # TODO: Insert your code to build the state graph here
    # *** example for adding neighbors ***
    # if path between n1 and n2 is collision free:
    #   n1.neighbors.append(n2)
    #   n2.neighbors.append(n1)
    vertices, edges = state_graph(350, 45, 5, init_node, goal_node, spec)

    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)
        # print(current.__hash__())
        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            # print(init_visited[current])
            print("found")
            # for i in init_visited[current]:
            #     print(str(i))
            write_robot_config_list_to_file(output, primitive_step(init_visited[current]))
            print("finish")
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                if test_self_collision(suc.config, spec) \
                        and test_environment_bounds(suc.config) \
                        and test_obstacle_collision(suc.config, spec, spec.obstacles):
                    init_container.append(suc)
                    init_visited[suc] = init_visited[current] + [suc.config]


def state_graph(n, k, r, init, goal, spec):
    """
    Construct a graph by performing Probabilistic Roadmap.
    :param n: number of nodes to put in the roadmap
    :param k: number of closest neighbors to examine for each configuration
    :param r: the maximum distance between each configurations (in radians)
    :param init: the initial configuration
    :param spec: the specification of the problem
    :return: (list, list) vertices and edges that represents the graph.
    """
    vertices = []
    edges = []
    current = init.config
    while len(vertices) < n:
        if current.ee1_grappled:
            collision = True
            while collision:
                angles = []
                lengths = []
                for i in current.ee1_angles:
                    angles.append(Angle(degrees=random.randint(-165, 165)))

                if spec.min_lengths[0] != spec.max_lengths[0]:
                    for i in range(len(spec.min_lengths)):
                        lengths.append(random.uniform(spec.min_lengths[i], spec.max_lengths[i]))
                else:
                    lengths = current.lengths

                next_node = make_robot_config_from_ee1(
                    current.points[0][0],
                    current.points[0][1],
                    angles,
                    lengths,
                    ee1_grappled=True,
                )

                if test_obstacle_collision(next_node, spec, spec.obstacles):
                    vertices.append(GraphNode(spec, next_node))
                    collision = False

    vertices.append(init)
    vertices.append(goal)
    for q in vertices:
        closest = find_closest(q, k, vertices)
        for target in closest:
            if (q, target) not in edges:
                iteration_local_planner(q, target, spec)
                ###
                # edges.append((target, q))

    return vertices, edges

# def convert_to_degrees(rad_angle):
#     return rad_angle * 180 / math.pi


def primitive_step(configs):
    result = []
    result.append(configs[0])

    for x in range(len(configs) - 1):
        current_angles = configs[x].ee1_angles
        next_angles = configs[x+1].ee1_angles

        current_lengths = configs[x].lengths
        next_lengths = configs[x+1].lengths

        abs_difference_angles = []
        abs_difference_lengths = []

        difference_angles = []
        difference_lengths = []

        # length of lengths and angles are the same
        for y in range(len(current_angles)):
            diff_angle = next_angles[y].in_radians() - current_angles[y].in_radians()
            abs_diff_angle = abs(diff_angle)
            abs_difference_angles.append(abs_diff_angle)

            difference_angles.append(diff_angle)

            diff_length = next_lengths[y] - current_lengths[y]
            abs_diff_length = abs(diff_length)
            abs_difference_lengths.append(abs_diff_length)

            difference_lengths.append(diff_length)

        # print("length : " + str(abs_difference_lengths))
        # print("angle : " + str(abs_difference_angles))
        segments = max(max(abs_difference_angles), max(abs_difference_lengths))
        # print("segments : " + str(segments))
        # print()

        configs_needed = int(math.ceil(segments/0.001))

        direction_vector_angles = []
        direction_vector_lengths = []
        for dv in range(len(abs_difference_angles)):
            direction_vector_angles.append(difference_angles[dv] / configs_needed)
            direction_vector_lengths.append(difference_lengths[dv] / configs_needed)

        # print("dvdirection_vector_angles)
        for step in range(configs_needed):
            angles = []
            lengths = []

            for i in range(len(direction_vector_angles)):
                angle = Angle(radians=current_angles[i].in_radians() + (step * direction_vector_angles[i]))
                angles.append(angle)

                length = current_lengths[i] + (step * direction_vector_lengths[i])
                lengths.append(length)

            new_config = make_robot_config_from_ee1(configs[0].get_ee1()[0], configs[0].get_ee1()[1], angles,
                                                    lengths, configs[0].ee1_grappled, ee2_grappled=False)
            result.append(new_config)
        result.append(configs[x+1])
    return result


def find_closest(q, k, vertices):
    """
    Find k vertices that are closest to q.
    :param q: vertex being examined
    :param k: number of closest neighbors to examine for each configuration
    :param vertices: list of vertices
    :return: (list) list of vertices closest to q with the size of k.
    """
    new_arr = vertices[:]
    new_arr.remove(q)

    new_arr.sort(key=lambda config: distance_angle(config, q))

    return new_arr[:k]


def iteration_local_planner(source, target, spec):
    init = source
    goal = target

    mid_ = create_mid(source, target, spec)
    root = Node(mid_)
    queue = [(source, target, root)]

    for i in range(25):
        source, target, mid = queue.pop(0)

        child_left = create_mid(source, mid.val, spec)
        child_right = create_mid(mid.val, target, spec)
        mid.left = Node(child_left)
        mid.right = Node(child_right)

        children = [(source, mid.val, mid.left), (mid.val, target, mid.right)]
        for child in children:
            queue.append(child)

    lst = Node.in_order(root)

    init.neighbors.append(lst[0])
    lst[0].neighbors.append(init)

    for k in range(len(lst) - 1):
        lst[k].neighbors.append(lst[k+1])
        lst[k+1].neighbors.append(lst[k])

    lst[k+1].neighbors.append(goal)
    goal.neighbors.append(lst[k+1])

    return True


def create_mid(q, target, spec):
    mid_angles = []
    mid_lengths = []
    for i, j in zip(q.config.ee1_angles, target.config.ee1_angles):
        k = (i.in_degrees() + j.in_degrees()) / 2
        mid_angles.append(Angle(degrees=k))

    for i, j in zip(q.config.lengths, target.config.lengths):
        k = (i + j) / 2
        mid_lengths.append(k)

    mid_conf = make_robot_config_from_ee1(
        q.config.points[0][0],
        q.config.points[0][1],
        mid_angles,
        mid_lengths,
        ee1_grappled=True,
    )

    return GraphNode(spec, mid_conf)


def get_tip(q):
    if q.config.ee1_grappled:
        return q.config.points[-1]
    else:
        return q.config.points[0]


def get_nearest_obstacle(q, obstacles):
    lst = []
    p = get_tip(q)
    for o in obstacles:
        for e in o.edges:
            x0 = p[0]
            y0 = p[1]
            x1 = e[0][0]
            y1 = e[0][1]
            x2 = e[1][0]
            y2 = e[1][1]
            dist = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1) / math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            lst.append(dist)
    return min(lst)


def get_nearest_obstacle_angle(q, obstacles):
    lst = []
    p = get_tip(q)
    for o in obstacles:
        for e in o.edges:
            x0 = p[0]
            y0 = p[1]
            x1 = e[0][0]
            y1 = e[0][1]
            x2 = e[1][0]
            y2 = e[1][1]
            yf = abs(y1 - y2)
            xf = abs(x1 - x2)
            # print("yf = " + str(yf))
            # print("xf = " + str(xf))
            dist = Angle.atan2(yf, xf)
            # print("dist = " + str(dist))
            lst.append(dist)
    return min(lst)


def distance(q1, q2):
    """
    Find the distance between two vertices.
    :param q1: vertex as the first operand
    :param q2: vertex as the second operand
    :return: (double) the distance between the two vertices.
    """
    # return abs(q1.config.angle_sum() - q2.config.angle_sum())
    q1_tip = get_tip(q1)
    q2_tip = get_tip(q2)
    x1 = q1_tip[0]
    y1 = q1_tip[1]
    x2 = q2_tip[0]
    y2 = q2_tip[1]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def distance_angle(q1, q2):
    return min(abs(q1.config.angle_sum() - q2.config.angle_sum()),
               360 - (abs(q1.config.angle_sum() - q2.config.angle_sum())))


def main(args):
    spec = ProblemSpec(args[0])
    solve(spec, args[1])


if __name__ == '__main__':
    start = timeit.default_timer()
    main(sys.argv[1:])
    print("Solved in " + str(timeit.default_timer() - start))
