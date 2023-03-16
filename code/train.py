import torch                                    # 导入torch
import torch.nn as nn                           # 导入torch.nn
import torch.nn.functional as F                 # 导入torch.nn.functional
import numpy as np                              # 导入numpy
import argparse
import copy

from run_experiments import import_mapf_instance
from agent import Agent

# 超参数
BATCH_SIZE = 32                                 # 样本数量
LR = 0.01                                       # 学习率
EPSILON = 0.1                                   # greedy policy
GAMMA = 0.9                                     # reward discount: close to 0 weights more on immediate and close 1 weight more on future reward
TARGET_REPLACE_ITER = 100                       # 目标网络更新频率
MEMORY_CAPACITY = 1000                          # 记忆库容量
N_ACTIONS = 5                                   # there are 5 action: left, right, up, down, stay
N_STATES = 28                                   # an agent state include the agent position and map state


"""
torch.nn是专门为神经网络设计的模块化接口。nn构建于Autograd之上，可以用来定义和运行神经网络。
nn.Module是nn中十分重要的类，包含网络各层的定义及forward方法。
定义网络：
    需要继承nn.Module类，并实现forward方法。
    一般把网络中具有可学习参数的层放在构造函数__init__()中。
    只要在nn.Module的子类中定义了forward函数，backward函数就会被自动实现(利用Autograd)。
"""

# action map: 0: up, 1: down, 2: right, 3: left, 4: stay
Action_map = {0: [1, 0], 1: [-1, 0], 2: [0, 1], 3: [0, -1], 4: [0, 0]}
Index_action_map = {(1, 0): 0, (-1, 0): 1, (0, 1): 2, (0, -1): 3, (0, 0): 4}

def print_map(s):
    map, pos = s
    map[pos[0]][pos[1]] = 3
    for i in range(len(map)):
        print(map[i])


# Reset the environment
# Train single agent and by default train the first agent
# Maybe changed later
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
        self.fc1 = nn.Linear(N_STATES, 50)                                      # input layer: input should be map size
        self.fc1.weight.data.normal_(0, 0.1)                                    # normalization with average 0 and sdv with 0.1
        self.fc2 = nn.Linear(50, N_ACTIONS)                                     # hidden layer
        self.fc2.weight.data.normal_(0, 0.1)                                    # normalization

    def forward(self, x):
        x = F.relu(self.fc1(x))
        actions_value = self.fc2(x)
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

        # Define the compound data type for the transition
        transition_dtype = np.dtype([
            ('state', np.float64, (1, N_STATES)),
            ('action', np.int64, (1, 2)),
            ('reward', np.float64),
            ('next_state',  np.float64, (1, N_STATES)),
            ('done', bool)
        ])
        self.memory = np.zeros((MEMORY_CAPACITY,), dtype=transition_dtype)      # initialize the memory, each row is a transition

    def choose_action(self, x):
        if np.random.uniform() < EPSILON:                                       # choose network best action or random action base on epsilon
            actions_value = self.eval_net.forward(x)                            # get action value from target net work
            action_index = int(torch.argmax(actions_value))                     # get the index of most possible action
        else:                                                                   # random action
            action_index = np.random.randint(0, N_ACTIONS)                      # random action of the five action

        return Action_map[action_index]

    # Memory cache function used to store transition, each inout is a transition
    def store_transition(self, s, a, r, s_, done):
        s_ = torch.FloatTensor(s_).view(1, -1)                                  # change the s_ shape to a row: 1*64
        # if the memory is full, overwrite the old transition
        index = self.memory_counter % MEMORY_CAPACITY                           # 获取transition要置入的行数

        self.memory[index]['state'] = s
        self.memory[index]['action'] = Index_action_map[tuple(a)]               # store the action index instead of action
        self.memory[index]['reward'] = r
        self.memory[index]['next_state'] = s_
        self.memory[index]['done'] = done

        self.memory_counter += 1                                                # memory_counter自加1

    # Learning evaluate network to update the target network
    def learn(self):
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:                  # after 100 evaluate network learning
            self.target_net.load_state_dict(self.eval_net.state_dict())         # update the target network
        self.learn_step_counter += 1

        # random select batch size of transition
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)            # random select a batch size index over [0, 2000]

        b_memory = self.memory[sample_index]                                    # get the corresponding transition

        #b_s = torch.FloatTensor(b_memory["state"])
        b_s = torch.from_numpy(np.copy(b_memory["state"])).float()
        b_a = torch.from_numpy(np.copy(b_memory["action"])).long()
        b_r = torch.from_numpy(np.copy(b_memory["reward"])).float()
        b_s_ = torch.from_numpy(np.copy(b_memory["next_state"])).float()

        # 获取32个transition的评估值和目标值，并利用损失函数和优化器进行评估网络参数更新
        # get the evaluate q value
        q_eval = self.eval_net(b_s).squeeze().gather(1, b_a)                    # squeeze the dim from [32, 1, 5] to [32, 5]

        # eval_net(b_s)通过评估网络输出32行每个b_s对应的一系列动作值，然后.gather(1, b_a)代表对每行对应索引b_a的Q值提取进行聚合
        q_next = self.target_net(b_s_).detach()
        # q_next不进行反向传递误差，所以detach；q_next表示通过目标网络输出32行每个b_s_对应的一系列动作值
        q_target = b_r + GAMMA * q_next.max(1)[0].view(BATCH_SIZE, 1)
        # q_next.max(1)[0]表示只返回每一行的最大值，不返回索引(长度为32的一维张量)；.view()表示把前面所得到的一维张量变成(BATCH_SIZE, 1)的形状；最终通过公式得到目标值
        loss = self.loss_func(q_eval, q_target)
        # 输入32个评估值和32个目标值，使用均方损失函数
        self.optimizer.zero_grad()                                      # 清空上一步的残余更新参数值
        loss.backward()                                                 # 误差反向传播, 计算参数更新值
        self.optimizer.step()                                           # 更新评估网络的所有参数


def train(map, starts, agent):
    dqn = DQN()                                                         # initialize a dqn
    init_map = env_reset(map, starts, agent)                            # initial state: map, starts, goals

    for i in range(1000):                                               # 1000 episode loop
        print('<<<<<<<<<Episode: %s' % i)
        cur_map = copy.deepcopy(init_map)                               # reset environment
        agent.reset_agent()                                             # reset the agent

        # state consists of current map(block,other agents as block) and position
        cur_map[agent.pos[0]][agent.pos[1]] = 3                         # 3 indicates the current position in the map
        s = torch.FloatTensor(cur_map).view(1, -1)                      # matrx to a row indicate the state

        while True:                                                     # start an episode (each loop indicates a step)
            a = dqn.choose_action(s)                                    # input the current state and choose an action
            s_, r, done = agent.nextStep(a, cur_map)                    # conduct action and require feedback,

            cur_map, cur_pos = s_                                       # unwrap the next state
            cur_map[cur_pos[0]][cur_pos[1]] = 3                         # 3 indicates the current position in the map
            s_ = torch.FloatTensor(cur_map).view(1, -1)                 # matrx to a row indicate the state

            dqn.store_transition(s, a, r, s_, done)                     # store transition

            s = s_                                                      # update state
            print(dqn.memory_counter)
            if dqn.memory_counter > MEMORY_CAPACITY:                    # trigger learning if 2000 memory capacity full
                # start learning, (random select 32 transition and update eval_net)
                # after 100 times learning, assign the eval_net to target_net
                dqn.learn()

            if done:                                                    # if finished
                break


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

    train(my_map, starts, agent)