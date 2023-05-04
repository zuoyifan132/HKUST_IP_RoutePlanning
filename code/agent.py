import copy
import config

class Agent:
    def __init__(self, index, pos, goal):
        self.index = index
        self.start = copy.deepcopy(pos)
        self.goal = goal
        self.pos = pos  # [x, y]
        self.collide = False

    # detect collision base on current agent position and current map.
    # return finish or not and corresponding reward.
    # reward: agent achieve goal: +100, collision: -5, go_next: -0.5
    def detect_finish(self, map, goal, heuristic):
        # collide with block
        if map[self.pos[0]][self.pos[1]] == 2:
            print("COLLIDE WITH BLOCK")
            return [True, config.collide_with_block]
        # collide with agent
        elif map[self.pos[0]][self.pos[1]] == 3:
            print("COLLIDE WITH AGENT")
            return [True, config.collide_with_agent]
        # reach goal
        elif self.pos[0] == goal[0] and self.pos[1] == goal[1]:
            print("REACH GOAL!")
            return [True, config.reach_goal]
        # does not collide
        else:
            # manhattan distance heuristic
            return [False, config.doesnt_collide + heuristic]

    def nextStep(self, action, map):
        # calculate manhattan distance heuristic before conduct action
        pre_manhattan_dist = (self.pos[0] - self.goal[0])**2 + (self.pos[1] - self.goal[1])**2

        # set the previous position to free: 1
        # after conduct the action, the occupied pos should be 1
        map[self.pos[0]][self.pos[1]] = 1

        # conduct the action change position
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]

        # calculate manhattan distance heuristic after conduct action
        after_manhattan_dist = (self.pos[0] - self.goal[0])**2 + (self.pos[1] - self.goal[1])**2

        # calculate manhattan distance heuristic
        heuristic = pre_manhattan_dist - after_manhattan_dist

        done, r = self.detect_finish(map, self.goal, heuristic)

        # if not done, change the current position to 100
        if not done:
            map[self.pos[0]][self.pos[1]] = 100

        return [[map, self.pos], r, done]

    def reset_agent(self):
        self.pos = copy.deepcopy(self.start)
        self.collide = False

    def print_agent(self):
        print("Agent number: ", self.index)
        print("Agent start: ", self.start)
        print("Agent position: ", self.pos)
        print("Agent goal: ", self.goal)
        print("Agent collide: ", self.collide)
