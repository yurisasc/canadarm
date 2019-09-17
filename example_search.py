from tester import test_config_equality

"""
Example of code for performing BFS on a state graph, including a class representing a state graph node.

COMP3702 2019 Assignment 2 Support Code

Last updated by njc 17/09/19
"""


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

    def get_successors(self):
        return self.neighbors



def solve(spec):
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


    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]




