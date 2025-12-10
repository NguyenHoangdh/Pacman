from game import Agent, Actions, Directions
import util
from searchAgents import PositionSearchProblem 
import search
import searchAgents 


def getPredictedPosition(pacmanPos, pacmanDirection, distance, state):
    """
    Tính vị trí dự đoán của Pac-Man sau 'distance' bước đi, dừng trước tường.
    """
    x, y = pacmanPos
    current_pos = (int(x), int(y))
    
    if pacmanDirection == Directions.STOP:
        return current_pos

    dx, dy = Actions.directionToVector(pacmanDirection)
    dx = int(dx)
    dy = int(dy)
    
    predicted_pos = current_pos
    walls = state.getWalls()

    for _ in range(distance):
        next_x = predicted_pos[0] + dx
        next_y = predicted_pos[1] + dy
        
        if walls[next_x][next_y]:
            break 
        
        predicted_pos = (next_x, next_y)
            
    return predicted_pos

class GhostAgent(Agent):
    def __init__(self, index):
        self.index = index

    def getAction(self, state):
        dist = self.getDistribution(state)
        if len(dist) == 0:
            return Directions.STOP
        else:
            return util.chooseFromDistribution(dist)

    def getDistribution(self, state):
        util.raiseNotDefined()

# 1. ChaserGhost (Truy Đuổi)
class ChaserGhost(GhostAgent):
    def __init__(self, index):
        super().__init__(index)

    def getAction(self, state):
        ghostPos = state.getGhostPosition(self.index)
        pacmanPos = state.getPacmanPosition()
        legalActions = state.getLegalActions(self.index)
        ghostState = state.getGhostState(self.index)
        isScared = ghostState.scaredTimer > 0

        if not legalActions:
            return Directions.STOP

        if isScared:
            return self._fallback_flee(ghostPos, pacmanPos, legalActions)
        else:
            problem = PositionSearchProblem(
                state,
                start=ghostPos,
                goal=pacmanPos,
                warn=False,
                visualize=False
            )
            path = search.aStarSearch(problem, heuristic=searchAgents.manhattanHeuristic)
            
            if len(path) > 0 and path[0] in legalActions:
                return path[0]
            else:
                return self._fallback_chase(ghostPos, pacmanPos, legalActions)

    def _fallback_chase(self, ghostPos, pacmanPos, legalActions):
        bestAction = None
        minDist = float('inf')
        for a in legalActions:
            dx, dy = Actions.directionToVector(a)
            nextPos = (ghostPos[0]+dx, ghostPos[1]+dy)
            dist = util.manhattanDistance(nextPos, pacmanPos)
            if dist < minDist:
                minDist = dist
                bestAction = a
        return bestAction

    def _fallback_flee(self, ghostPos, pacmanPos, legalActions):
        bestAction = None
        maxDist = -1
        for a in legalActions:
            dx, dy = Actions.directionToVector(a)
            nextPos = (ghostPos[0]+dx, ghostPos[1]+dy)
            dist = util.manhattanDistance(nextPos, pacmanPos)
            if dist > maxDist:
                maxDist = dist
                bestAction = a
        return bestAction

# 2. AmbusherGhost (Chặn Đầu)
class AmbusherGhost(GhostAgent):
    def __init__(self, index, prediction_distance=4):
        super().__init__(index)
        self.prediction_distance = prediction_distance 

    def getAction(self, state):
        ghostPos = state.getGhostPosition(self.index)
        pacmanState = state.getPacmanState()
        legalActions = state.getLegalActions(self.index)
        ghostState = state.getGhostState(self.index)
        isScared = ghostState.scaredTimer > 0
        pacmanPos = pacmanState.getPosition()
        
        if not legalActions:
            return Directions.STOP

        if isScared:
            return ChaserGhost(self.index)._fallback_flee(ghostPos, pacmanPos, legalActions)
        else:
            pacmanDirection = pacmanState.getDirection()
            goal_pos = getPredictedPosition(pacmanPos, pacmanDirection, self.prediction_distance, state)
            
            problem = PositionSearchProblem(
                state,
                start=ghostPos,
                goal=goal_pos,
                warn=False,
                visualize=False
            )
            path = search.aStarSearch(problem, heuristic=searchAgents.manhattanHeuristic)
            
            if len(path) > 0 and path[0] in legalActions:
                return path[0]
            else:
                return ChaserGhost(self.index)._fallback_chase(ghostPos, pacmanPos, legalActions)