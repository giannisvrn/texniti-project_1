# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

# from asyncore import loop
# from msilib.schema import Error
# from msilib.schema import Error
# from sre_parse import State
# from importlib.resources import path
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), '', 0, [])
    fringe = util.Stack()
    fringe.push(node)
    explored = set()

    while(True):
        if( fringe.isEmpty()):
            return None
        node = fringe.pop()
        if( problem.isGoalState(node[0]) == True):
            return node[3]
        explored.add(node[0])
        l = problem.getSuccessors(node[0])
        for i in l:
            not_in = True
            for j in explored:
                if( i[0] == j):
                    not_in = False
            if( not_in == True):
                path = node[3] + [i[1]]
                temp_node = (i[0], i[1], i[2], path)
                fringe.push(temp_node)
    # util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), '', 0, [])
    fringe = util.Queue()
    fringe.push(node)
    explored = set()
    neighbors = set()
    neighbors.add(node[0])

    while(True):
        if( fringe.isEmpty()):
            return None
        node = fringe.pop()
        if( problem.isGoalState(node[0]) == True):
            return node[3]
        explored.add(node[0])
        l = problem.getSuccessors(node[0])
        for i in l:
            not_in = True
            not_in_geitones = True
            for j in explored:
                if( i[0] == j):
                    not_in = False
            for j in neighbors:
                if( i[0] == j):
                    not_in_geitones = False
            if( not_in == True and not_in_geitones == True):
                path = node[3] + [i[1]]
                temp_node = (i[0], i[1], i[2], path)
                fringe.push(temp_node)
                neighbors.add(temp_node[0])
    # util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), '', 0, [],0)
    fringe = util.PriorityQueue()
    fringe.push(node,node[4])
    explored = set()
    neighbors = set()
    neighbors.add((node[0],node[4]))         # Kratame kai to path_cost twn neighbors

    while(True):
        if( fringe.isEmpty()):
            return None
        node = fringe.pop()
        explored.add(node[0])
        if( problem.isGoalState(node[0]) == True):
            return node[3]
        l = problem.getSuccessors(node[0])
        for i in l:
            child = (i[0],i[1],i[2],node[3] + [i[1]], problem.getCostOfActions(node[3]+[i[1]]))
            not_in = True
            not_in_geitones = True
            for j in explored:
                if( child[0] == j):
                    not_in = False
            for j in neighbors:
                if( child[0] == j[0]):
                    not_in_geitones = False
                    break
            if( not_in == True and not_in_geitones == True):         # ean den yparxei to state tote to prosthetoume sto frontier kai stous neighbors
                fringe.push(child,child[4])
                neighbors.add((child[0],child[4]))
            else:
                if( child[4] < j[1]):      # ean to path to neighbor einai megalytero apo to kainourio path tote eisagoume to kainourio
                    fringe.push(child,child[4])

    # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
