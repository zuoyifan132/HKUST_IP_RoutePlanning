import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import argparse
import copy
from run_experiments import import_mapf_instance
from visualize import Animation
import config
from agent import Agent
from train import env_reset, read_agents
from agent import all_agent_move

# hyper parameters
BATCH_SIZE = config.BATCH_SIZE
LR = config.LR
EPSILON = 0.99
GAMMA = config.GAMMA
TARGET_REPLACE_ITER = config.TARGET_REPLACE_ITER
MEMORY_CAPACITY = config.MEMORY_CAPACITY
NUM_ACTION = config.NUM_ACTION
NUM_AGENT = config.NUM_AGENT
N_ACTIONS = NUM_ACTION**NUM_AGENT
N_STATES = config.N_STATES
EPISODE = config.EPISODE


def print_map(s):
    for i in range(len(s)):
        res = list(map(int, s[i]))
        print(res)
    print()


# 定义Net类 (定义网络)
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(N_STATES, 20)                  # input layer: input should be map size
        self.fc1.weight.data.normal_(0, 0.1)                # normalization with average 0 and sdv with 0.1
        self.fc2 = nn.Linear(20, 40)
        self.fc2.weight.data.normal_(0, 0.1)
        self.fc3 = nn.Linear(40, N_ACTIONS)                  # output layer
        self.fc3.weight.data.normal_(0, 0.1)                # normalization

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        actions_value = self.fc3(x)
        return actions_value


def choose_action(target_net, x):
    if np.random.uniform() < EPSILON:  # choose network best action or random action base on epsilon
        actions_value = target_net.forward(x)  # get action value from target net work
        best_actions_indexes = int(torch.argmax(actions_value))  # get the index of most possible action foe all agents
    else:
        best_actions_indexes = np.random.randint(1, N_ACTIONS)  # random action of the five action

    return config.Action_map[best_actions_indexes]


def find_solution(agents, map):
    all_path = []
    net = Net()
    net.load_state_dict(torch.load("target_net.pth"))       # load the trained state

    cur_map = env_reset(map, agents)

    print_map(cur_map)
    print()

    s = torch.FloatTensor(cur_map).view(1, -1)              # matrx to a row indicate the state

    while True:
        a = choose_action(net, s)                           # input the current state and choose an action
        print("action: ", a)
        s_, r, done = all_agent_move(agents, a, s)          # all agents conduct action and require feedback
        print("rewad: ", r)
        s_ = torch.FloatTensor(s_).view(config.map_height, config.map_width)
        print_map(s_.tolist())
        s_ = torch.FloatTensor(s_).view(1, -1)              # matrx to a row indicate the state
        s = s_                                              # update state

        if done:                                            # if finished
            break

    for agent in agents:
        print(str(agent.index) + " path: ", agent.path)
        all_path.append(agent.path)

    return all_path


# main function to run the training
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training the MAPF solver...')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    args = parser.parse_args()
    filename = args.instance
    my_map, starts, goals = import_mapf_instance(filename)
    _, agents = read_agents(filename)

    path = find_solution(agents, copy.deepcopy(my_map))
    print("all path: ", path)

    animation = Animation(my_map, starts, goals, path)
    animation.show()
