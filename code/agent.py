import copy
import config
import torch


def print_map(s):
    for i in range(len(s)):
        print(s[i])
    print()

def heuristic_fine_tune(heuristic):
    # stay
    if heuristic == 0:
        return config.doesnt_collide
    else:
        heuristic = config.heuristic_fine_tune_weight / heuristic
        return  heuristic

# all agent move base on action
# only when all agent reach goal or collide with block, the episode is done
def all_agent_move(agents, a, map):
    r = 0
    done = False
    num_reach_goal = 0
    reward_list = []

    # convert 1 dimension map to 10*10
    map = torch.FloatTensor(map).view(config.map_height, config.map_width)

    for i in range(len(agents)):
        map, each_r, each_done, reach_goal = agents[i].nextStep(a[i], map)

        reward_list.append((each_r, reach_goal))

        # each agent done
        if each_done:
            # reach goal
            if reach_goal:
                # agent reach goal
                num_reach_goal += 1
            # collide
            else:
                done = True

    # There are agent reach goal
    if num_reach_goal > 0:
        # all agent reach goal
        if num_reach_goal == config.NUM_AGENT:
            done = True
            r = sum([r for r, _ in reward_list])
        # partially reach goal, change reward to partial reward
        else:
            for re, rg in reward_list:
                # skip the reach goal
                if not rg:
                    r += re
    # no agent reach goal
    else:
        r = sum([r for r, _ in reward_list])
    print("reward list: ", reward_list)

    return [map, r, done]

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
    # return finish or not, reach goal or not and corresponding reward.
    # reward: agent achieve goal: +100, collision: -5, go_next: -0.5
    def detect_finish(self, map, goal, heuristic, action, pre_pos):
        # collide with block
        if map[self.pos[0]][self.pos[1]] == 2:
            print(str(self.index), " COLLIDE WITH BLOCK")
            self.collide = True
            return [True, False, config.collide_with_block]
        # collide with agent
        elif map[self.pos[0]][self.pos[1]] > 100:
            print(str(self.index), " COLLIDE WITH AGENT")
            self.collide = True
            return [True, False, config.collide_with_agent]
        # reach goal
        elif self.pos[0] == goal[0] and self.pos[1] == goal[1]:
            # already reach goal, but still move reward should be 10 to keep stay in goal
            if action == (0, 0):
                print("agent " + str(self.index) + " previously reached goal and stay")
                return [True, True, heuristic]
            # first time reach goal
            else:
                print(str(self.index) + " pos is: " + str(self.pos) + " goal is: " + str(self.goal))
                return [True, True, config.reach_goal]
        # previously reach goal and leave goal get penalty
        elif pre_pos[0] == goal[0] and pre_pos[1] == goal[1]:
            if action != (0, 0):
                map[goal[0]][goal[1]] = 5
                return [False, False, heuristic]
        # does not collide
        else:
            # manhattan distance heuristic
            return [False, False, heuristic]

    def nextStep(self, action, map):
        my_map = copy.deepcopy(map)

        # calculate manhattan distance heuristic before conduct action
        pre_manhattan_dist = (self.pos[0] - self.goal[0])**2 + (self.pos[1] - self.goal[1])**2

        # set the previous position to free: 1
        # after conduct the action, the occupied pos should be 1
        my_map[self.pos[0]][self.pos[1]] = 1

        # conduct the action change position and add to path
        pre_pos = [self.pos[0],self.pos[1]]
        self.pos[0] = self.pos[0] + action[0]
        self.pos[1] = self.pos[1] + action[1]
        self.path.append(tuple(self.pos))

        # calculate manhattan distance heuristic after conduct action
        after_manhattan_dist = (self.pos[0] - self.goal[0])**2 + (self.pos[1] - self.goal[1])**2

        # calculate manhattan distance heuristic
        heuristic = pre_manhattan_dist - after_manhattan_dist
        heuristic = heuristic_fine_tune(heuristic)

        done, reach_goal, r = self.detect_finish(my_map, self.goal, heuristic, action, pre_pos)

        # if not done, change the current position to its index on map
        my_map[self.pos[0]][self.pos[1]] = self.index

        return [my_map, r, done, reach_goal]

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
