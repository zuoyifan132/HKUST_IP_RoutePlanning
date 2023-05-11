import copy
import config


# all agent move base on action
# only when all agent reach goal or collide with block, the episode is done
def all_agent_move(agents, a, my_map):
    r = 0
    done = False
    reach_goal = 0
    collide_agent = []
    for i in range(len(agents)):
        my_map, each_r, each_done = agents[i].nextStep(a[i], my_map)
        r += each_r

        # each agent done
        if each_done:
            # agent done but collide
            if each_r == config.collide_with_block:
                collide_agent.append(agents[i])
                # agent reach goal
            else:
                reach_goal += 1

    # all agent reach goal or collide with block, the episode is done
    if len(collide_agent) + reach_goal == config.NUM_AGENT:
        done = True

    return [my_map, r, done]

class Agent:
    def __init__(self, index, pos, goal):
        # the index also indicates the value of agent in map
        self.index = index
        self.start = copy.deepcopy(pos)
        self.goal = goal
        self.pos = pos  # [x, y]
        self.collide = False
        self.path = [tuple(self.start)]

    # detect collision base on current agent position and current map.
    # return finish or not and corresponding reward.
    # reward: agent achieve goal: +100, collision: -5, go_next: -0.5
    def detect_finish(self, map, goal, heuristic, action):
        # collide with block
        if map[self.pos[0]][self.pos[1]] == 2:
            print(str(self.index), " COLLIDE WITH BLOCK")
            self.collide = True
            return [True, config.collide_with_block]
        # collide with agent
        elif map[self.pos[0]][self.pos[1]] == 3:
            print(str(self.index), " COLLIDE WITH AGENT")
            self.collide = True
            return [True, config.collide_with_agent]
        # reach goal
        elif self.pos[0] == goal[0] and self.pos[1] == goal[1]:
            # already reach goal, but still move reward should be 0
            if action == (0, 0):
                return [True, 0]
            # first time reach goal
            else:
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

        # conduct the action change position and add to path
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]
        self.path.append(tuple(self.pos))

        # calculate manhattan distance heuristic after conduct action
        after_manhattan_dist = (self.pos[0] - self.goal[0])**2 + (self.pos[1] - self.goal[1])**2

        # calculate manhattan distance heuristic
        heuristic = (pre_manhattan_dist - after_manhattan_dist)* config.heuristic_weight
        done, r = self.detect_finish(map, self.goal, heuristic, action)

        # if not done, change the current position to 100
        if not done:
            map[self.pos[0]][self.pos[1]] = self.index

        return [map, r, done]

    def reset_agent(self):
        self.pos = copy.deepcopy(self.start)
        self.collide = False
        self.path = [self.start]

    def print_agent(self):
        print("Agent number: ", self.index)
        print("Agent start: ", self.start)
        print("Agent position: ", self.pos)
        print("Agent goal: ", self.goal)
        print("Agent collide: ", self.collide)
        print("Agent path: ", self.path)
