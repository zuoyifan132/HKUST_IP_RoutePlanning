import copy


class Agent:
    def __init__(self, index, pos, goal):
        self.index = index
        self.start = copy.deepcopy(pos)
        self.goal = goal
        self.pos = pos  # [x, y]
        self.collide = False

    # detect collision base on current agent position and current map.
    # return finish or not and corresponding reward.
    # reward: agent achieve goal: +1, collision: -10000000, go_next: -1
    def detect_finish(self, map, goal):
        # collide with block
        if map[self.pos[0]][self.pos[1]] == 2:
            print("COLLIDE WITH BLOCK")
            return [True, float(-100)]
        # collide with agent
        elif map[self.pos[0]][self.pos[1]] == 3:
            print("COLLIDE WITH AGENT")
            return [True, float(-100)]
        # reach goal
        elif self.pos[0] == goal[0] and self.pos[1] == goal[1]:
            print("REACH GOAL!")
            return [True, float(10000)]
        # does not collide
        else:
            return [False, float(-50)]

    def nextStep(self, action, map):
        # set the previous position to free: 1
        # after conduct the action, the occupied pos should be 1
        map[self.pos[0]][self.pos[1]] = 1

        # conduct the action change position
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]

        done, r = self.detect_finish(map, self.goal)

        # if not done, change the current position to 4
        if not done:
            map[self.pos[0]][self.pos[1]] = 4

        # train the agent that will not always wait
        if action == [0, 0]:
            r = -2

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
