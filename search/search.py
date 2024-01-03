# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import heapq


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "START OF PERSONAL CODE"
    
    pStack = util.Stack()
    startState = problem.getStartState()
    path = []
    pStack.push(startState)
    parents = { startState : None}
    visited = [startState]
    while not pStack.isEmpty():
        currState = pStack.pop()
        if problem.isGoalState(currState):
            while parents[currState] is not None:
                currState, direction = parents[currState]
                path.append(direction)
            path.reverse()
            return path
        successors = problem.getSuccessors(currState)
        for successor in successors:
            if successor[0] not in visited:
                visited.append(successor[0])
                parents[successor[0]] = (currState, successor[1])
                pStack.push(successor[0])
    util.raiseNotDefined()

    "END OF PERSONAL CODE"

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "START OF PERSONAL CODE"
    
    pQueue = util.Queue()
    startState = problem.getStartState()
    path = []
    pQueue.push(startState)
    parents = { startState : None}
    visited = [startState]
    while not pQueue.isEmpty():
        currState = pQueue.pop()
        if problem.isGoalState(currState):
            while parents[currState] is not None:
                currState, direction = parents[currState]
                path.append(direction)
            path.reverse()
            return path
        successors = problem.getSuccessors(currState)
        for successor in successors:
            if successor[0] not in visited:
                visited.append(successor[0])
                parents[successor[0]] = (currState, successor[1])
                pQueue.push(successor[0])
    util.raiseNotDefined()

    "END OF PERSONAL CODE"

def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    "START OF PERSONAL CODE"

    pQueue = util.PriorityQueue()
    startState = problem.getStartState()
    pQueue.push(startState, 0)
    path = []
    visited = []
    parents = {startState : None}
    distance = {startState : 0}
    while not pQueue.isEmpty():
        currState = pQueue.pop()
        currCost = distance[currState]
        visited.append(currState)
        if problem.isGoalState(currState):
            while parents[currState] is not None:
                pState, direction = parents[currState]
                path.append(direction)
                currState = pState
            path.reverse()
            return path
        successors = problem.getSuccessors(currState)
        for sucState, direction, cost in successors:
            if sucState not in visited:
                if sucState in distance and distance[sucState] < cost + currCost:
                    continue
                if sucState in distance:
                    pQueue.push(sucState, cost + currCost)
                else:
                    pQueue.push(sucState, cost + currCost)
                distance[sucState] = cost + currCost
                parents[sucState] = (currState, direction)
    util.raiseNotDefined()
    
    "END OF PERSONAL CODE"

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def stateToString(state):
    corners = state[1]
    cornerString = ""
    for corner in corners:
        cornerString = cornerString + str(corner) + "  "
    return "Position: " + str(state[0]) + " | Corners Found: " + (cornerString)

def printSuccessors(successors):
    print("Successors: ")
    for succ in successors:
        pos = succ[0][0]
        corners = succ[0][1]
        print("[" + str(pos) + ", " + str(corners) + "]   ")
    print("\n")

def printList(list):
    for elem in list:
        print(stateToString(elem))
    print("\n")


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    "START OF PERSONAL CODE"

    pQueue = util.PriorityQueue()
    startState = problem.getStartState()
    pQueue.push(startState, 0)
    path = []
    visited = []
    parents = {startState : None}
    distance = {startState : 0}
    counter = 1
    while not pQueue.isEmpty():
        currState = pQueue.pop()
        currCost = distance[currState]
        visited.append(currState)
        if problem.isGoalState(currState):
            while parents[currState] is not None:
                pState, direction = parents[currState]
                path.append(direction)
                currState = pState
            path.reverse()
            return path
        successors = problem.getSuccessors(currState)
        for sucState, direction, cost in successors:
            if sucState not in visited:
                if sucState in distance and distance[sucState] < cost + currCost:
                    continue
                if sucState in distance:
                    pQueue.push(sucState, cost + currCost + heuristic(sucState, problem))
                else:
                    pQueue.push(sucState, cost + currCost + heuristic(sucState,problem))
                distance[sucState] = cost + currCost
                parents[sucState] = (currState, direction)
        counter += 1
        heapCpy = pQueue.heap
        heapStates = {}
        for entry in heapCpy:
            pri, ind, state = entry
            if state in heapStates and heapStates[state] < pri:
                continue
            heapStates[state] = pri
        i = 0
        newHeap = []
        for state in heapStates:
            entry = (heapStates[state], i, state)
            heapq.heappush(newHeap, entry)
            i += 1
        pQueue.heap = newHeap
    util.raiseNotDefined()

    "END OF PERSONAL CODE"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch