import util
from game import Directions
from typing import List
class SearchProblem:
    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []
    stk = util.Stack()
    visited = set()
    stk.push((start, []))
    while not stk.isEmpty():
        current_state, actions = stk.pop()
        if problem.isGoalState(current_state):
            return actions
        
        if current_state not in visited:
            visited.add(current_state)
            for next_state, atc, _ in problem.getSuccessors(current_state):
                if next_state not in visited:
                    stk.push((next_state, [*actions, atc]))
    return []

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []

    queue = util.Queue()
    visited = set()
    queue.push((start, []))
    while not queue.isEmpty():
        current_state, actions = queue.pop()
        if problem.isGoalState(current_state):
            return actions

        if current_state not in visited:
            visited.add(current_state)
            for next_state, atc, _ in problem.getSuccessors(current_state):
                if next_state not in visited:
                    queue.push((next_state, [*actions, atc]))
    return []

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []
    p_queue = util.PriorityQueue()
    visited = {start: 0}
    p_queue.push((start, [], 0), 0)
    while not p_queue.isEmpty():
        current_state, actions, cost = p_queue.pop()
        if problem.isGoalState(current_state):
            return actions

        for next_state, atc, step in problem.getSuccessors(current_state):
            next_cost = cost + step
            if next_state not in visited or visited[next_state] > next_cost:
                visited[next_state] = next_cost
                p_queue.push((next_state, [*actions, atc], next_cost), next_cost)
    return []

def nullHeuristic(state, problem=None) -> float:
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]: 
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []

    p_queue = util.PriorityQueue()
    visited = {start: 0}
    p_queue.push((start, [], 0), 0)

    while not p_queue.isEmpty():
        current_state, actions, cost = p_queue.pop()

        if problem.isGoalState(current_state):
            return actions

        for next_state, atc, step in problem.getSuccessors(current_state):
            next_cost = cost + step
            priority = next_cost + heuristic(next_state, problem)
            if next_state not in visited or visited[next_state] > next_cost:
                visited[next_state] = next_cost
                p_queue.push((next_state, [*actions, atc], next_cost), priority)
    return []

bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
astar = aStarSearch
