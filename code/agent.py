class Agent:
    def __init__(self, index, pos, goal):
        self.index = index
        self.pos = pos
        self.goal = goal
        self.collide = False

    def nextStep(self, action):
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]

        return self.pos

    def print_agent(self):
        print("Agent number: ", self.index)
        print("Agent position: ", self.pos)
        print("Agent goal: ", self.goal)
        print("Agent collide: ", self.collide)
