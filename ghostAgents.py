from game import Agent, Actions, Directions
import util
from searchAgents import PositionSearchProblem
import search

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

class AStarGhost(GhostAgent):
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
            bestAction = self._fallback_flee(ghostPos, pacmanPos, legalActions)
            return bestAction
        else:
            problem = PositionSearchProblem(
                state,
                start=ghostPos,
                goal=pacmanPos,
                warn=False,
                visualize=False
            )
            path = search.aStarSearch(problem)
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
