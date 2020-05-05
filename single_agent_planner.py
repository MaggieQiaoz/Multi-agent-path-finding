import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = {}
    for constraint in constraints:
        if constraint['positive']:
            if constraint['agent'] != agent:
                if constraint['timestep'] not in table:
                    table[constraint['timestep']] = []
                table[constraint['timestep']].append(
                    constraint['loc']
                    if len(constraint['loc']) == 1
                    else [constraint['loc'][1], constraint['loc'][0]])
        else:
            if constraint['agent'] == agent:
                if constraint['timestep'] not in table:
                    table[constraint['timestep']] = []
                table[constraint['timestep']].append(constraint['loc'])
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        if [next_loc] in constraint_table[next_time]:
            return True
        if [curr_loc, next_loc] in constraint_table[next_time]:
            return True
    # Code to handle Task 2.3 (Adding Additional Constraints)
    reached_goal_constraints = [
        (timestep, constraint_table[timestep])
        for timestep in constraint_table.keys()
        if timestep < 0]  # At goal = Negative timestep = Constrain for all future timesteps too
    for timestep, reached_goal_constraint in reached_goal_constraints:
        if next_time >= -timestep:
            if [next_loc] in reached_goal_constraint:
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def push_node_EPEA(open_list, node):
    duplicate=False
    for n in open_list:
        if node['F_val']==n[0] and node['f_val']==n[1] and node['h_val']==n[2] and node['loc']==n[3] and node['time']==n[4]:
            duplicate=True
            break
    if(duplicate==False):
           heapq.heappush(open_list, (node['F_val'], node['f_val'], node['h_val'], node['loc'], node['time'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def pop_node_EPEA(open_list):
    _, _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = max([
        constraint['timestep']
        for constraint in constraints
        if constraint['agent'] == agent and constraint['loc'] == [goal_loc]],
        default=-1) + 1
    open_cells = sum(not cell for row in my_map for cell in row)
    path_length_upper_bound = open_cells + len(constraints)
    constraint_table = build_constraint_table(constraints, agent)
    h_value = h_values[start_loc]
    root = {
        'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'parent': None,
        'time': 0
    }
    push_node(open_list, root)
    closed_list[(root['loc'], root['time'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['time'] > path_length_upper_bound:
            continue
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['time'] >= earliest_goal_timestep:
            return get_path(curr)
        if not is_constrained(curr['loc'], curr['loc'], curr['time'] + 1, constraint_table):
            wait_child = {
                'loc': curr['loc'],
                'g_val': curr['g_val'] + 1,
                'h_val': curr['h_val'],
                'parent': curr,
                'time': curr['time'] + 1
            }
            if (wait_child['loc'], wait_child['time']) in closed_list:
                existing_node = closed_list[(wait_child['loc'], wait_child['time'])]
                if compare_nodes(wait_child, existing_node):
                    closed_list[(wait_child['loc'], wait_child['time'])] = wait_child
                    push_node(open_list, wait_child)
            else:
                closed_list[(wait_child['loc'], wait_child['time'])] = wait_child
                push_node(open_list, wait_child)
        for dir in range(4):
            child_loc = move(curr['loc'], dir)
            if not (0 <= child_loc[0] < len(my_map)) or not (
                    0 <= child_loc[1] < len(my_map[0])) or my_map[child_loc[0]][child_loc[1]]:
                continue
            if not is_constrained(curr['loc'], child_loc, curr['time'] + 1, constraint_table):
                child = {
                    'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time': curr['time'] + 1
                }
                if (child['loc'], child['time']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['time'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['time'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['time'])] = child
                    push_node(open_list, child)

    return None  # Failed to find solutions

def OFS(my_map, curr, h_values, constraint_table):
    table=dict()
    child_list=[]
    F_next=float('inf')
    if not is_constrained(curr['loc'], curr['loc'], curr['time'] + 1, constraint_table):
        table[1]=([4], 2)
    dir0=[]
    dir2=[]
    for dir in range(4):
        child_loc = move(curr['loc'], dir)
        if not (0 <= child_loc[0] < len(my_map)) or not ( 0 <= child_loc[1] < len(my_map[0])) or my_map[child_loc[0]][child_loc[1]]:
            continue
        if not is_constrained(curr['loc'], child_loc, curr['time'] + 1, constraint_table):
            if((curr['g_val'] + 1 + h_values[child_loc])-curr['f_val']==0):
                dir0.append(dir)
            else:
                dir2.append(dir)
    if dir0!=[]:
        if 1 in table:
            table[0]=(dir0, 1)
        elif dir2!=[]:
            table[0]=(dir0, 2)
        else:
            table[0]=(dir0, float('inf'))
    if dir2!=[]:
        table[2]=(dir2, float('inf'))
    else:
        table[1]=([4], float('inf'))
    diff=curr['F_val']-curr['f_val']
    if diff in table:
        child_list=table[diff][0]
        F_next=table[diff][1]+curr['f_val']
    elif diff+1 in table:
        F_next=table[diff+1][1]+curr['f_val']
    elif diff+2 in table:
        F_next=table[diff+2][1]+curr['f_val']

    return child_list, F_next


def EPEA(my_map, start_loc, goal_loc, h_values, agent, constraints):
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = max([
        constraint['timestep']
        for constraint in constraints
        if constraint['agent'] == agent and constraint['loc'] == [goal_loc]],
        default=-1) + 1
    open_cells = sum(not cell for row in my_map for cell in row)
    path_length_upper_bound = open_cells + len(constraints)
    constraint_table = build_constraint_table(constraints, agent)
    h_value = h_values[start_loc]
    root = {
        'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'parent': None,
        'time': 0,
        'f_val': h_value,
        'F_val': h_value
    }
    push_node_EPEA(open_list, root)
    while len(open_list) > 0:
        curr = pop_node_EPEA(open_list)
        if curr['time'] > path_length_upper_bound:
            continue
        if curr['loc'] == goal_loc and curr['time'] >= earliest_goal_timestep:
            return get_path(curr)
        child_list, F_next=OFS(my_map, curr, h_values, constraint_table)
        if child_list==[]:
            curr['F_val']=F_next
            if F_next!=float('inf'):
                push_node_EPEA(open_list, curr)
            continue
        for dir in child_list:
            if dir in range(4): child_loc = move(curr['loc'], dir)
            else: child_loc= curr['loc']
            child = {
                    'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time': curr['time'] + 1,
                    'f_val':curr['g_val'] + 1 + h_values[child_loc],
                    'F_val':curr['g_val'] + 1 + h_values[child_loc]
                }
            push_node_EPEA(open_list, child)
                
        if F_next==float('inf'):
            closed_list[(curr['loc'], curr['time'])] = curr
        else:
            curr['F_val']=F_next
            push_node_EPEA(open_list, curr)
    return None  # Failed to find solutions
