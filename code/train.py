import sys

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import argparse
import copy

from run_experiments import import_mapf_instance
from agent import Agent
import config
from agent import all_agent_move

# hyper parameters
BATCH_SIZE = config.BATCH_SIZE
LR = config.LR
EPSILON = config.EPSILON
GAMMA = config.GAMMA
TARGET_REPLACE_ITER = config.TARGET_REPLACE_ITER
MEMORY_CAPACITY = config.MEMORY_CAPACITY
NUM_ACTION = config.NUM_ACTION
NUM_AGENT = config.NUM_AGENT
N_ACTIONS = NUM_ACTION**NUM_AGENT
N_STATES = config.N_STATES
EPISODE = config.EPISODE


# print the mao
def print_map(s):
    for i in range(len(s)):
        res = list(map(int, s[i]))
        print(res)
    print()

def print_statistic(reach_goal_rate_list):
    print()
    print("------------------------------Statistic------------------------------")
    for each in reach_goal_rate_list:
        print(each)

# read all agents
def read_agents(file_name):
    my_map, starts, goals = import_mapf_instance(file_name)
    agents = []

    for i in range(len(starts)):
        pos = list(starts[i])
        agent = Agent(i+config.self_agent, pos, goals[i])
        agents.append(agent)

    return [my_map, agents]

# Reset the environment
def env_reset(my_map, agents):
    # set free grid and block
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            my_map[i][j] = config.free_grid if not my_map[i][j] else config.block

    for agent in agents:
        # treat self as 100
        my_map[agent.start[0]][agent.start[1]] = agent.index
        # set goal in map
        my_map[agent.goal[0]][agent.goal[1]] = config.goal
        # reset agent
        agent.reset_agent()

    return my_map


# Net class (define network)
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


# Define DQN (define two network)
# Experience Replay
class DQN(object):
    def __init__(self):
        self.eval_net, self.target_net = Net(), Net()                           # evaluate network and target network
        self.learn_step_counter = 0                                             # for target updating
        self.memory_counter = 0                                                 # for storing memory
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=LR)    # adam optimizer
        self.loss_func = nn.MSELoss()                                           # mean square error loss

        # load the state to the target network if the path is not None
        try:
            self.target_net.load_state_dict(torch.load("target_net.pth"))
        except:
            print("No target_net.pth file, train from scratch")

        # Define the compound data type for the transition
        transition_dtype = np.dtype([
            ('state', np.float64, (1, N_STATES)),
            ('actions', np.int64),
            ('reward', np.float64),
            ('next_state',  np.float64, (1, N_STATES)),
            ('done', bool)
        ])
        self.memory = np.zeros((MEMORY_CAPACITY,), dtype=transition_dtype)      # initialize the memory, each row is a transition

    # choose action for each agent
    def choose_action(self, x):
        if np.random.uniform() < EPSILON:                                       # choose network best action or random action base on epsilon
            actions_value = self.eval_net.forward(x)                            # get action value from target net work
            best_actions_indexes = int(torch.argmax(actions_value))             # get the index of most possible action foe all agents
        else:
            best_actions_indexes = np.random.randint(1, N_ACTIONS)              # random action of the five action

        return [config.Action_map[best_actions_indexes], best_actions_indexes]

    # Memory cache function used to store transition, each inout is a transition
    def store_transition(self, s, a_i, r, s_, done):
        s_ = torch.FloatTensor(s_).view(1, -1)                                  # change the s_ shape to a row: 1*64
        # if the memory is full, overwrite the old transition
        index = self.memory_counter % MEMORY_CAPACITY                           # if memory is full, overwrite the old memory with new memory

        self.memory[index]['state'] = s
        self.memory[index]['reward'] = r
        self.memory[index]['next_state'] = s_
        self.memory[index]['done'] = done
        self.memory[index]['actions'] = a_i                                     # store the action index instead of action

        self.memory_counter += 1                                                # memory_counter + 1

    # Learning evaluate network to update the target network
    def learn(self):
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:                  # after 100 evaluate network learning
            self.target_net.load_state_dict(self.eval_net.state_dict())         # update the target network
        self.learn_step_counter += 1

        # random select batch size of transition
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)            # random select a batch size index over [0, 2000]

        b_memory = self.memory[sample_index]                                    # get the corresponding transition

        b_s = torch.from_numpy(np.copy(b_memory["state"])).float().squeeze()
        b_a = torch.from_numpy(np.copy(b_memory["actions"])).long().unsqueeze(0)# use un-squeeze to change [32] to [[32, 5], 1]
        b_r = torch.from_numpy(np.copy(b_memory["reward"])).float().squeeze()
        b_s_ = torch.from_numpy(np.copy(b_memory["next_state"])).float().squeeze()
        b_d = torch.from_numpy(np.copy(b_memory["done"])).bool().squeeze()

        # get the evaluate q value
        q_eval = self.eval_net(b_s).gather(1, b_a.t()).squeeze()                # get the corresponding value of Q_value
        q_next = self.target_net(b_s_).detach().squeeze()
        q_target = (b_r + GAMMA * q_next.max(1)[0].unsqueeze(0)).squeeze()

        # if b_d is True, the q_target is b_r
        for i in range(BATCH_SIZE):
            if b_d[i]:
                q_target[i] = b_r[i]

        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()                                      # clear the last stage remaining updating parameter
        loss.backward()                                                 # calculate loss
        self.optimizer.step()                                           # update the neural nets parameters


def train(map, agents):
    global EPSILON
    total_reach_gaol = {}                                               # total reach goal number
    reach_goal_rate = []                                                # each epsilon rate goal rate
    dqn = DQN()                                                         # initialize a dqn
    init_map = env_reset(map, agents)                                   # initial state: map, starts, goals

    while EPSILON < 0.9:                                                # number of episode loop
        total_reach_gaol[str(EPSILON)] = 0                              # initialize each epsilon total reach goal

        print("------------------EPSILON: %s------------------" % EPSILON)
        num_episode_each_epsilon = int(EPISODE*(1-EPSILON))
        for i in range(num_episode_each_epsilon):                       # number of episode loop
            print('<<<<<<<<<Episode: %s' % i)
            cur_map = copy.deepcopy(init_map)                           # reset environment

            for agent in agents:                                        # reset the agents
                agent.reset_agent()
                # state consists of current map(block,other agents as block) and position
                cur_map[agent.pos[0]][agent.pos[1]] = agent.index       # agent index indicates the current position in the map

            s = torch.FloatTensor(cur_map).view(1, -1)                  # matrix to a row indicate the state

            while True:                                                 # start an episode (each loop indicates a step)
                a, a_i = dqn.choose_action(s)                           # input the current state output joint actions and action index
                print("action: ", a)
                s_, r, done = all_agent_move(agents, a, s)              # all agents conduct action and require feedback
                print("reward: ", r)
                print_map(s_)
                s_ = torch.FloatTensor(s_).view(1, -1)                  # matrix to a row indicate the state\
                dqn.store_transition(s, a_i, r, s_, done)               # store transition
                s = s_                                                  # update state

                if dqn.memory_counter > MEMORY_CAPACITY:                # trigger learning if 2000 memory capacity full
                    # start learning, (random select 32 transition and update eval_net)
                    # after 100 times learning, assign the eval_net to target_net
                    dqn.learn()

                if done:                                                # if finished
                    count = 0
                    for agent in agents:
                        if tuple(agent.pos) == agent.goal:
                            count += 1
                    if count == NUM_AGENT:                              # if reach goal
                        print("All REACH GOAL")
                        total_reach_gaol[str(EPSILON)] += 1
                    break

        reach_goal_rate.append("For EPSILON " +str(round(EPSILON, 1)) + ", Reach goal rate: " + str(total_reach_gaol[str(EPSILON)] / num_episode_each_epsilon))

        EPSILON = EPSILON + 0.1                                         # increase the epsilon

    print_statistic(reach_goal_rate)                                    # print the reach goal rate statistic

    # save the net state
    torch.save(dqn.target_net.state_dict(), "target_net.pth")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training the MAPF solver...')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    args = parser.parse_args()
    filename = args.instance
    my_map, agents = read_agents(filename)
    train(my_map, agents)