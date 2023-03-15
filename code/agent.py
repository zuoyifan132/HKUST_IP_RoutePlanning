class Agent:
    def __init__(self, index, pos, goal):
        self.index = index
        self.pos = pos          # [x, y]
        self.goal = goal
        self.collide = False

    # detect collision base on current agent position and current map.
    # return finish or not and corresponding reward.
    # reward: agent achieve goal: +0, collision: -inf, go_next: -1
    def detect_finish(self, map, goal):
        # collide with block
        if map[self.pos[0]][self.pos[1]] == 1:
            return [True, float("-inf")]
        # collide with agent
        elif map[self.pos[0]][self.pos[1]] == 2:
            return [True, float("-inf")]
        # reach goal
        elif self.pos[0] == goal[0] and self.pos[1] == goal[1]:
            return [True, float(0)]
        # does not collide
        else:
            return [False, float(-1)]

    def nextStep(self, action, map, goal):
        # conduct the action
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]

        done, r = self.detect_collision(map)

        return [[map, self.pos], r, done]

    def print_agent(self):
        print("Agent number: ", self.index)
        print("Agent position: ", self.pos)
        print("Agent goal: ", self.goal)
        print("Agent collide: ", self.collide)
