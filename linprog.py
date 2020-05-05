import numpy as np
import networkx as nx
import itertools
import matplotlib.pyplot as plt
import pyomo.environ as pyo
from pyomo.opt import SolverFactory, check_optimal_termination
from typing import *


def manhattan(u: Tuple[int, int],
              v: Tuple[int, int]) -> int:
    """
    Manhattan distance
    :param u: First point
    :param v: Second point
    :return: Manhattan distance between u and v
    """
    return abs(u[0] - v[0]) + abs(u[1] - v[1])


class LPSolver:
    def __init__(self,
                 map: List[List[bool]],
                 starts: List[Tuple[int, int]],
                 goals: List[Tuple[int, int]]):
        """
        Initialize an LP solver for MAPF
        :param map: True if cell is an obstacle, False if cell is free
        :param starts: Agent start locations
        :param goals: Agent goal locations
        """
        # Create graph
        self.map = np.array(map)
        self.obstacles = [coords for coords, val in np.ndenumerate(self.map) if val]
        self.G_undirected = nx.grid_2d_graph(*self.map.shape)
        self.G_undirected.remove_nodes_from(self.obstacles)
        self.G = self.G_undirected.to_directed()
        self.G.add_edges_from((v, v) for v in self.G.nodes)
        # Create bounding parameters
        # Bounds from Surynek, Felner, Stern, Boyarski:
        # Efficient SAT Approach to Multi-Agent Path Finding under the Sum of Costs Objective
        # https://docs.wixstatic.com/ugd/749b4b_c7d1fe5a70a84dbe8f52a6695e45457b.pdf
        self.individual_costs = [nx.astar_path_length(self.G, start, goal, manhattan)
                                 for start, goal in zip(starts, goals)]
        self.xi_0 = sum(self.individual_costs)  # Lower bound on minimal sum-of-costs
        self.mu_0 = max(self.individual_costs)  # Lower bound on minimal makespan
        # Save agent starts and goals
        self.starts = starts
        self.goals = goals
        # Initialize LP solver
        self.opt = SolverFactory('glpk')

    def delta(self, xi: int) -> int:
        """
        Extra cost over sum of individual costs (xi_0)
        :param xi: Desired sum-of-costs of solution
        :return: Delta (xi - xi_0)
        """
        return xi - self.xi_0

    def mu(self, xi: int) -> int:
        """
        Required number of time expansions
        :param xi: Desired sum-of-costs of solution
        :return: Mu (mu_0 + delta(xi))
        """
        return self.mu_0 + self.delta(xi)

    def time_expand(self, mu: int) -> nx.DiGraph:
        """
        Time expand the graph
        :param mu: Number of time expansions
        :return: Time expanded graph
        """
        # MAPF via Reduction slides, "Compact Formulation"
        T = nx.tensor_product(self.G, nx.path_graph(mu + 1, create_using=nx.DiGraph))
        # Prune vertices not on a start -> goal path for any agent
        # This finds a union of the MDDs (which are subgraphs of the TEG)
        keep = set(itertools.chain.from_iterable(
            (nx.descendants(T, start) | {start}) & (nx.ancestors(T, goal) | {goal})
            for start, goal in zip(zip(self.starts, itertools.repeat(0, len(self.starts))),
                                   zip(self.goals, itertools.repeat(mu, len(self.goals))))))
        T.remove_nodes_from(T.nodes - keep)
        nx.relabel_nodes(T, {v: (*v[0], v[1]) for v in T.nodes}, copy=False)
        return T

    def find_solution(self) -> Tuple[List[List[Tuple[int, int]]], float]:
        """
        Run an LP solver for MAPF
        :return: Path for each agent
        """
        # Guard against trivial case
        starts = self.starts
        goals = self.goals
        num_agents = len(starts)
        agents = list(range(num_agents))
        solve_time = 0.0
        if starts == goals:
            return [[starts[agent]] for agent in agents], solve_time
        # Start search for a solution at the lowest possible sum-of-cost (xi = xi_0)
        agents_path = []
        for xi in itertools.count(self.xi_0):
            print('Searching xi = %d' % xi)
            mu = self.mu(xi)  # Required number of time expansions
            T = self.time_expand(mu)  # Time expanded graph
            model = pyo.ConcreteModel()
            model.nodes = pyo.Set(dimen=3, initialize=T.nodes)
            model.edges = pyo.Set(within=model.nodes * model.nodes, initialize=T.edges)
            model.agents = pyo.Set(initialize=agents)
            model.x = pyo.Var(model.agents, model.edges,
                              domain=pyo.NonNegativeIntegers,
                              initialize=0)
            model.obj = pyo.Objective(expr=pyo.summation(
                model.x,
                index={(agent, e)
                       for agent in agents
                       for e in T.in_edges((*goals[agent], mu))}),
                sense=pyo.maximize)
            model.capacity = pyo.ConstraintList()
            for e in T.edges:
                model.capacity.add(
                    pyo.summation(
                        model.x,
                        index={(agent, e) for agent in agents}) <= 1)
            model.conservation = pyo.ConstraintList()
            model.sources = pyo.ConstraintList()
            model.sinks = pyo.ConstraintList()
            model.vertex_collisions = pyo.ConstraintList()
            for v in T.nodes:
                if T.succ[v]:
                    model.vertex_collisions.add(
                        pyo.summation(
                            model.x,
                            index={(agent, e) for e in T.out_edges(v) for agent in agents}) <= 1)
                for agent in agents:
                    if v == (*starts[agent], 0):
                        model.sources.add(
                            pyo.summation(
                                model.x,
                                index={(agent, e) for e in T.out_edges(v)}) == 1)
                        continue
                    if v == (*goals[agent], mu):
                        model.sinks.add(
                            pyo.summation(
                                model.x,
                                index={(agent, e) for e in T.in_edges(v)}) == 1)
                        continue
                    model.conservation.add(
                        pyo.summation(
                            model.x,
                            index={(agent, e) for e in T.in_edges(v)}) -
                        pyo.summation(
                            model.x,
                            index={(agent, e) for e in T.out_edges(v)}) == 0)
            model.edge_collisions = pyo.ConstraintList()
            for u, v in T.edges:
                if u > v and ((*v[:2], u[2]), (*u[:2], v[2])) in T.edges:
                    model.edge_collisions.add(
                        pyo.summation(
                            model.x,
                            index={(agent, x, u[2], y, v[2])
                                   for agent, (x, y) in itertools.product(
                                    agents,
                                    itertools.permutations([u[:2], v[:2]]))}
                        ) <= 1
                    )
            model.extra_edges = pyo.Constraint(expr=pyo.summation(
                model.x,
                index={(agent, u, v)
                       for agent, (u, v) in itertools.product(agents, T.edges)
                       if u[2] in range(self.individual_costs[agent], mu)
                       and not u[:2] == v[:2] == goals[agent]}
            ) == self.delta(xi))
            sum_of_costs = float('inf')
            while sum_of_costs != xi:
                results = self.opt.solve(model, timelimit=60)
                solve_time += results.solver.time
                if check_optimal_termination(results):
                    agent_attr = {e: agent
                                  for e in T.edges
                                  for agent in agents
                                  if pyo.value(model.x[agent, e])}
                    nx.set_edge_attributes(T, values=agent_attr, name='agent')
                    moves = sorted((data['agent'], u[-1], u[:-1], v[:-1])
                                   for u, v, data in T.edges(data=True)
                                   if 'agent' in data.keys())
                    agents_moves = [[move[2:] for move in moves]
                                    for k, moves in itertools.groupby(moves, key=lambda x: x[0])]
                    agents_path = [[agent_move[0] for agent_move in agent_moves] + [agent_moves[-1][-1]]
                                   for agent_moves in agents_moves]
                    agents_path = [
                        list(
                            reversed(
                                [agent_path[-1]] + list(itertools.dropwhile(lambda x: x == agent_path[-1],
                                                                            reversed(agent_path)))
                            )
                        )
                        for agent_path in agents_path
                    ]
                    sum_of_costs = sum(len(agent_path) - 1 for agent_path in agents_path)
                    if sum_of_costs != xi:
                        # Rare: more than one solution with exactly delta extra edges; an agent waited at its goal
                        # then later departed it, meaning there is a non-extra edge which should be an extra edge.
                        # Specifically, the non-extra edge "wait" action at the goal is actually an extra edge.
                        # The following technique was attempted here:
                        # Pivot to another solution by cutting out the current solution's combination of edges.
                        # Since each solution is uniquely defined by its combination of edges, we add a constraint
                        # making the current combination of edges infeasible, forcing a new solution.
                        # However, it did not work.
                        # We output a message informing of the existence of the solution
                        # but regrettably return a suboptimal one.
                        print('***RARE: GLPK returned another solution, see comments!!!***')
                        print('Solve time: %f' % solve_time)
                        print('Sum of costs (GLPK\'s solution): %d' % sum_of_costs)
                        print('Sum of costs (Actual): %d' % xi)
                        print('Makespan (mu): %d' % mu)
                        print('Extra edges (delta): %d' % self.delta(xi))
                        print('Lower bound on sum of costs (xi_0): %d' % self.xi_0)
                        print('Due to technical limitations, GLPK\'s solution will be shown')
                        sum_of_costs = xi
                    else:
                        print('***Found solution***')
                        print('Solve time: %f' % solve_time)
                        print('Sum of costs (xi): %d' % xi)
                        print('Makespan (mu): %d' % mu)
                        print('Extra edges (delta): %d' % self.delta(xi))
                        print('Lower bound on sum of costs (xi_0): %d' % self.xi_0)
                else:
                    break
            if sum_of_costs == xi:
                break
        return agents_path, solve_time
