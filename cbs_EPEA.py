import time as timer
import heapq
import random
import itertools
from single_agent_planner import compute_heuristics, EPEA, get_location, get_sum_of_cost, a_star


# def pairwise(iterable):
#     a, b = itertools.tee(iterable)
#     next(b, None)
#     return zip(a, b)


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    # if get_location(path1, 0) == get_location(path2, 0):
    #     return [path1[0]], 0
    # edges = zip(pairwise(path1), pairwise(path2))
    # for timestep, (edges1, edges2) in enumerate(edges, 1):
    #     if edges1[1] == edges2[1]:
    #         return [edges1[1]], timestep
    #     if edges1 == list(reversed(edges2)):
    #         return list(edges1), timestep
    for timestep in range(max(len(path1), len(path2))):
        if get_location(path1, timestep) == get_location(path2, timestep):
            return [get_location(path1, timestep)], timestep
        if timestep + 1 < min(len(path1), len(path2)):
            if [get_location(path1, timestep),
                get_location(path1, timestep + 1)] == [get_location(path2, timestep + 1),
                                                       get_location(path2, timestep)]:
                return [get_location(path1, timestep),
                        get_location(path1, timestep + 1)], timestep + 1
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for a1, a2 in itertools.combinations(range(len(paths)), 2):
        detected = detect_collision(paths[a1], paths[a2])
        if detected:
            loc, timestep = detected
            collisions.append({'a1': a1, 'a2': a2, 'loc': loc, 'timestep': timestep})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if len(collision['loc']) == 1:
        return [
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': False
            },
            {
                'agent': collision['a2'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': False
            }]
    else:
        return [
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': False
            },
            {
                'agent': collision['a2'],
                'loc': [collision['loc'][1], collision['loc'][0]],
                'timestep': collision['timestep'],
                'positive': False
            }]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agent = random.choice([collision['a1'], collision['a2']])
    return [
        {
            'agent': agent,
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'positive': True
        },
        {
            'agent': agent,
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'positive': False
        }]


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolverEPEA(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = EPEA(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        split_func = disjoint_splitting if disjoint else standard_splitting
        while self.open_list:
            p = self.pop_node()
            # print(p['paths'])
            # print(p['constraints'])
            # print(p['collisions'])
            if not p['collisions']:
                solve_time = self.print_results(p)
                return p['paths'], solve_time
            collision = p['collisions'][0]
            constraints = split_func(collision)
            # print(constraints)

            for constraint in constraints:
                q = {  # 'cost': 0,
                     'constraints': p['constraints'] + [constraint]
                     if constraint not in p['constraints']
                     else list(p['constraints']),
                     'paths': list(p['paths']),
                     # 'collisions': []
                }
                if constraint['positive']:
                    agents = paths_violate_constraint(constraint, q['paths'])
                    paths_exist = True
                    for i in agents:
                        path = EPEA(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                      i, q['constraints'])
                        if not path:
                            paths_exist = False
                            break
                        q['paths'][i] = path
                    if paths_exist:
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.push_node(q)
                else:
                    i = constraint['agent']
                    path = EPEA(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                  i, q['constraints'])
                    if path:
                        q['paths'][i] = path
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.push_node(q)

        raise BaseException('No solutions')


    def print_results(self, node) -> float:
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        return CPU_time
