from initiation_to_research.MCTSClass import MCTSNode, MCTSClass
from math import log, sqrt
from copy import deepcopy
import numpy as np


""" Implementation of the abstract Node class for use in MCTS """


# -------------------------------------------------------------------------------------------


class MCTSPlannerClass(MCTSNode):  # MCTS Node, not a ROS2 Node

    def __init__(self, targets, posx, posy, goals=None, cost=0, mcts=None, assigned=None):
            self.targets = targets  # position of targets (list of 2D row vectors)
            self.posx, self.posy = posx, posy # agents position copied to avoid pointer isues (list of 2D row vectors)
            self.mcts = mcts if mcts is not None else MCTSClass(1/sqrt(8))  # initialize MCTS tree with exploration weight
            self.goals = goals if goals is not None else []
            self.cost = cost

            if assigned is not None: self.assigned = assigned
            else: self.assigned = [False for _ in range(len(self.targets))]

    def find_random_child(self):  # for simulating

        if self.is_terminal(): return None  # if the game is finished then no moves can be made

        # make a copy of everything that will be modified during the process
        posx, posy, cost, assigned, goals = self.posx, self.posy, self.cost, deepcopy(self.assigned), deepcopy(self.goals)

        # find the closest not assigned target
        pos = np.array([[posx], [posy]])
        distances = [sqrt(sum((x-pos)**2)) for x in self.targets]
        ind = np.argmin(distances)
        while self.assigned[ind]:
            distances[ind] = float('inf')
            ind = np.argmin(distances)

        # update the goal ,the cost, and move the agent position to the target position
        goals.append(self.targets[ind])
        cost = cost + distances[ind]
        assigned[ind] = True
        posx, posy = self.targets[ind][0, 0], self.targets[ind][1, 0]

        # create a new node with the new configuration
        return MCTSPlannerClass(targets=self.targets, posx=posx, posy=posy, goals=goals, cost=cost, mcts=self.mcts, assigned=assigned)

    def find_children(self):  # for expanding

        if self.is_terminal(): return None  # if the game is finished then no moves can be made

        # make a copy of everything that will be modified during the process
        posx, posy, cost, assigned, goals = self.posx, self.posy, self.cost, deepcopy(self.assigned), deepcopy(self.goals)

        # find all the children reachable from the current node

        children = []
        for i in range(len(self.targets)):
            if not self.assigned[i]:
                # make a copy of everything that will be modified during the process
                child_posx, child_posy, child_cost, child_goals, child_assigned = posx, posy, cost, deepcopy(goals), deepcopy(assigned)

                pos = np.array([[child_posx], [child_posy]])

                # update the goal ,the cost, and move the agent position to the target position
                child_goals.append(self.targets[i])
                child_cost = child_cost + sqrt(sum((self.targets[i]-pos)**2))
                child_assigned[i] = True
                child_posx, child_posy = self.targets[i][0, 0], self.targets[i][1, 0] 

                # create a new node with the new configuration and add it to the list of children of the current node
                child_node = MCTSPlannerClass(self.targets, child_posx, child_posy, child_goals, child_cost, self.mcts, child_assigned)
                children.append(child_node)

        # return the children nodes if there are any as a dict with no keys
        return {x for x in children} if len(children) > 0 else {self}

    def rollout(self, n):  # train the mcts tree n times to find possible next nodes to evolve in the game
        for _ in range(n): self.mcts.do_rollout(self)

    def choose(self):  # make a choice for the next node
        self = self.mcts.choose(self)
        return self

    def reward(self):  # the highest reward will be for the shortest overall cost
        return 1/self.cost

    def is_terminal(self):  # when all targets are assigned, the board is terminal
        is_terminal = False not in self.assigned
        # if terminal node, add the cost to go back to the origin
        if is_terminal: self.cost = self.cost + sqrt(sum((np.array([[0.], [0.]]) - np.array([[self.posx], [self.posy]]))**2))
        return is_terminal

    def __hash__(self):  # make up some hashing technique to make nodes comparable
        a = [tuple.__hash__(tuple([tuple(x.flatten().tolist()) for x in self.goals]))]
        return tuple.__hash__(tuple(a))

    def __eq__(self, other):  # same
        a = [tuple.__hash__(tuple([tuple(x.flatten().tolist()) for x in self.goals]))]
        b = [tuple.__hash__(tuple([tuple(x.flatten().tolist()) for x in other.goals]))]
        return tuple.__hash__(tuple(a)) == tuple.__hash__(tuple(b))
