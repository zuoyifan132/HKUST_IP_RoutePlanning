import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import argparse
import copy


from run_experiments import import_mapf_instance
from visualize import Animation

from agent import Agent

# 超参数
BATCH_SIZE = 32
LR = 0.01
EPSILON = 0.1
GAMMA = 0.9           # reward discount: cse to 0 weights more on immediate and close 1 weight more on future reward
TARGET_REPLACE_ITER = 100
MEMORY_CAPACITY = 1000
N_ACTIONS = 5         # there are 5 action: left, right, up, down, stay
N_STATES = 100        # an agent state include the agent position and map state


# action map: 0: up, 1: down, 2: right, 3: left, 4: stay
Action_map = {0: [1, 0], 1: [-1, 0], 2: [0, 1], 3: [0, -1], 4: [0, 0]}
Index_action_map = {(1, 0): 0, (-1, 0): 1, (0, 1): 2, (0, -1): 3, (0, 0): 4}

def print_map(s):
    map, pos = s
    map[pos[0]][pos[1]] = 3
    for i in range(len(map)):
        print(map[i])


def env_reset(my_map, starts, agent):
    #----------------------------------------------#

    # TODO:
    # Train each agent

    # ---------------------------------------------#

    # True as 1 and False as 0
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            my_map[i][j] = 0 if not my_map[i][j] else 1

    # treat other agents as block indicated by 2, true indicates block, exclude itself as block
    for x,y in starts[1:]:
        my_map[x][y] = 2

    # reset agent
    agent.reset_agent()

    return my_map


# 定义Net类 (定义网络)
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(N_STATES, 50)                  # input layer: input should be map size
        self.fc1.weight.data.normal_(0, 0.1)                # normalization with average 0 and sdv with 0.1
        self.fc2 = nn.Linear(50, N_ACTIONS)                 # hidden layer
        self.fc2.weight.data.normal_(0, 0.1)                # normalization

    def forward(self, x):
        x = F.relu(self.fc1(x))
        actions_value = self.fc2(x)
        return actions_value


def choose_action(target_net, x):
    if np.random.uniform() < EPSILON:                       # choose network best action or random action base on epsilon
        actions_value = target_net.forward(x)               # get action value from target net work
        action_index = int(torch.argmax(actions_value))     # get the index of most possible action
    else:                                                   # random action
        action_index = np.random.randint(0, N_ACTIONS)      # random action of the five action

    return Action_map[action_index]


def find_solution(agent, starts, map):
    all_path = []
    net = Net()
    net.load_state_dict(torch.load("target_net.pth"))       # load the trained state

    cur_map = env_reset(map, starts, agent)

    cur_map[agent.pos[0]][agent.pos[1]] = 3                 # 3 indicates the current position in the map
    s = torch.FloatTensor(cur_map).view(1, -1)              # matrx to a row indicate the state

    path = [tuple(agent.pos)]                               # the agent path

    while True:
        a = choose_action(net, s)                           # input the current state and choose an action
        s_, r, done = agent.nextStep(a, cur_map)            # conduct action and require feedback,

        cur_map, cur_pos = s_                               # unwrap the next state
        cur_map[cur_pos[0]][cur_pos[1]] = 3                 # 3 indicates the current position in the map
        s_ = torch.FloatTensor(cur_map).view(1, -1)         # matrx to a row indicate the state

        path.append(tuple(agent.pos))

        s = s_  # update state

        if done:  # if finished
            break

    all_path.append(path)
    # other agents should be empty for now, changed later
    for i in range(1, len(starts)):
        dup_path = []
        for j in range(len(path)):
            dup_path.append(starts[i])
        all_path.append(dup_path)

    return all_path


# main function to run the training
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training the MAPF solver...')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    args = parser.parse_args()
    filename = args.instance

    index = 0
    my_map, starts, goals = import_mapf_instance(filename)
    pos = list(starts[index])
    agent = Agent(index, pos, goals[index])

    path = find_solution(agent, copy.deepcopy(starts), copy.deepcopy(my_map))

    print("all path: ", path)

    animation = Animation(my_map, starts, goals, path)
    animation.show()
