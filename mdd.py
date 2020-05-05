import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import heapq
import copy 


class MDDSolver(object):
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
        print("mdd")

    def find_solution(self):

        print("finding solution")
        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
        print("result is")
        print(result)


        agentLength = []

        for res in result:
            agentLength.append(len(res))
        print(agentLength)
        search_list = [(agentLength)]
        close_search_list = dict()
        noSolution = False
        print('searched')
        print(search_list)
        count = 0
        while not noSolution:
            

            searching = search_list.pop(0)
            print(searching)

            #ASSUME 2 AGENTS FOR NOW
            mddAgent1 = generateMDD(self.starts[0], self.goals[0],self.my_map, searching[0])
            mddAgent2 = generateMDD(self.starts[1], self.goals[1],self.my_map, searching[1])
            # printMDD(mddAgent1)
            # print("____________________")
            # printMDD(mddAgent2)
            # print("*********************")

            mddFinal = fuseMDD(mddAgent1, mddAgent2)
            print(self.goals)
            goalnode = traverseTree(mddFinal,(self.goals[0],self.goals[1]))

            if goalnode:
                # print("found goal!!!")
                # print(searching)
                # print(goalnode['loc'])
                path = [goalnode['loc']]

                while goalnode['parent']:
                    path.append(goalnode['parent'][0]['loc'])
                    goalnode = goalnode['parent'][0]

                path.reverse()
                # print(path)

                result = [[],[]]

                for ele in path:
                    result[0].append(ele[0])
                    result[1].append(ele[1])
                          
                break

            # if tehres a soultion then break
            # else increment and keep going

            for incrementPosition in range(len(searching)):
                child = copy.deepcopy(searching) 
                child[incrementPosition] += 1
                tupChild = tuple(child)
                if tupChild not in close_search_list:
                    search_list.append(child)
                    close_search_list[tupChild] = True

            # print(search_list)


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(searching)))

        return result

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(ictNode):
    total = 0
    for value in ictNode:
        total += value
    return total

def traverseTree(node,goal):
    if node['loc'] == goal:
        return  node

    for child in node['children'].values():
        node = traverseTree(child, goal)
        if node:
            return node


        




def generateMDD(start_loc, goal_loc, my_map, cost):
    #breathfirst search from start loc for up to cost until it reaches goal_loc
    open_list = []
    closed_list = dict()
    root = {'loc':start_loc,'parent':[],'children': {}, 'cost': 0}
    # print(root)
    open_list.append(root)
    closed_list[root['loc']] = root
    test = 0

    while len(open_list) > 0:
        node = open_list.pop(0)
        if node['cost'] >= cost:
            break
        for dir in range(5):
            child_loc = move(node['loc'],dir)

            if not (0 <= child_loc[0] < len(my_map)) or not (0 <= child_loc[1] < len(my_map[0])) or my_map[child_loc[0]][child_loc[1]]:
                continue

            #if the child already exists at the cost level then add parent to the child

            if (child_loc , node['cost'] + 1) in closed_list:
                closed_list[(child_loc , node['cost'] + 1)]['parent'].append(node)
                node['children'][child_loc] = closed_list[(child_loc , node['cost'] + 1)]

            else:
                child = {
                        'loc': child_loc,
                        'parent': [node],
                        'children':{},
                        'cost': node['cost'] + 1
                    }
                node['children'][child_loc] = child

                closed_list[(child_loc , node['cost'] + 1)] = child
                open_list.append(child)

    print("done MDD")
    # printMDD(root)
    return root




def printMDD(root):
    
    queue = [root]
    # print(root)
    open_list = dict()
    print("******************************")
    while queue:
        node = queue.pop(0)

       
        print("cost: " + str(node['cost']) + " - loc: " + str(node['loc']))
        for child in node['children'].values():
            print("children: " + str(child['loc']))
            cost = node['cost'] + 1
            if (child['loc'], node['cost'] + 1) in open_list:
                continue
            else:
                queue.append(child)
                open_list[(child['loc'], node['cost'] + 1)] = True
    print("*********************************")


def fuseMDD(MDD1, MDD2):
    # print('fusing')

    open_list1 =[MDD1]
    open_list2 =[MDD2]
    root = None
    lookup = dict()

    nextlevel1 = []
    nextlevel2 = []


    while open_list1 or open_list2:
        nextlevel1 = []
        nextlevel2 = []


        for node1 in open_list1:
            for node2 in open_list2:
                assert(node1['cost'] == node2['cost'])
                
                if root == None:
                   
                    root = {
                        'loc': (node1['loc'],node2['loc']),
                        'parent': [],
                        'children':{},
                        'cost': 0
                    }
                    lookup[((node1['loc'],node2['loc']),node1['cost'])] = root

                    # nextlevel1 = node1['children'].values()
                    # nextlevel2 = node2['children'].values()
                    # print(node2['children'])
                else:
                    # print("entered else")
                    if node1['loc'] == node2['loc']: #check for conflict if both locations are the same
                        continue
                    else:
                        parent = []
                        parCheck = set()
                        for par1 in node1['parent']:
                            for par2 in node2['parent']:
                                assert(par1['cost'] == par2['cost'])
                                if ((par1['loc'],par2['loc']), par1['cost']) in lookup:
                                    if (par1['loc'],par2['loc']) not in parCheck :
                                        if (par2['loc'],par1['loc']) == (node1['loc'],node2['loc']):
                                            continue
                                        parent.append(lookup[((par1['loc'],par2['loc']), par1['cost'])])
                                        parCheck.add((par1['loc'],par2['loc']))
                        cost = max(node1['cost'],node2['cost'])
                        
                        if not parent:
                            continue

                        node = {
                        'loc': (node1['loc'],node2['loc']),
                        'parent': parent,
                        'children': {},
                        'cost':cost
                        }

                        lookup[((node1['loc'],node2['loc']),cost)] = node
                        for par in parent:
                            # print("parent")
                            # print(par)
                            if node['loc'] not in par['children']:
                                par['children'][node['loc']] = node
        childcheck = set()
        for node2 in open_list2:
            for child in node2['children'].values():
                if child['loc'] not in childcheck:
                    childcheck.add(child['loc'])

                    nextlevel2.append(child)
                        # print(node2['children'])
                # print('nextlevel')
                # print(nextlevel2)
        childcheck = set()
        for node1 in open_list1:
            for child in node1['children'].values():
                if child['loc'] not in childcheck:
                    childcheck.add(child['loc'])

                    nextlevel1.append(child)


        if nextlevel2 and not nextlevel1: #no childeren in one list then use previous level. openlist1 will be used again. create children for each element in list to show it waited
            # print("!!! first mdd done appending more!!!")
            open_list2 = nextlevel2

            for node in open_list1:
                temp_list = []
                temp_node ={
                'loc': node['loc'],
                'parent': [node],
                'children':{},
                'cost':node['cost'] + 1
                }
                # if isinstance(node['children'],dict):
                node['children'][node['loc']] = temp_node
                # else:
                #     node['children'].append(node)
                temp_list.append(temp_node)
            open_list1 = temp_list

        elif nextlevel1 and not nextlevel2:
            # print("!!! second mdd done appending more!!!")
            # print(len(nextlevel2))
            open_list1 = nextlevel1
            for node in open_list2:
                temp_list = []
                temp_node ={
                'loc': node['loc'],
                'parent': [node],
                'children':{},
                'cost':node['cost'] + 1
                }
                # if isinstance(node['children'],dict):

                # print(node['cost'])
                # print(temp_node['cost'])
                    # print(node['children'].keys())

                node['children'][node['loc']] = temp_node

                # testcount=True
                # else:
                    # node['children'].append(node)
                temp_list.append(temp_node)
            open_list2 = temp_list
            # print(open_list2)
        else:
            open_list1 = nextlevel1
            open_list2 = nextlevel2
        # print(open_list1)
        # print(open_list2)
        # print(open_list1 or open_list2)


    # print(root)

    # printMDD(root)
    return root

