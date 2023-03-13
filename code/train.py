import torch                                    # 导入torch
import torch.nn as nn                           # 导入torch.nn
import torch.nn.functional as F                 # 导入torch.nn.functional
import numpy as np                              # 导入numpy
import argparse

from run_experiments import import_mapf_instance
from agent import Agent

# 超参数
BATCH_SIZE = 32                                 # 样本数量
LR = 0.01                                       # 学习率
EPSILON = 0.9                                   # greedy policy
GAMMA = 0.9                                     # reward discount: close to 0 weights more on immediate and close 1 weight more on future reward
TARGET_REPLACE_ITER = 100                       # 目标网络更新频率
MEMORY_CAPACITY = 2000                          # 记忆库容量
N_ACTIONS = 5                                   # there are 5 action: left, right, up, down, stay
N_STATES = 0                                    # an agent state include the agent position and map state


"""
torch.nn是专门为神经网络设计的模块化接口。nn构建于Autograd之上，可以用来定义和运行神经网络。
nn.Module是nn中十分重要的类，包含网络各层的定义及forward方法。
定义网络：
    需要继承nn.Module类，并实现forward方法。
    一般把网络中具有可学习参数的层放在构造函数__init__()中。
    只要在nn.Module的子类中定义了forward函数，backward函数就会被自动实现(利用Autograd)。
"""

"""
function need to implemenrt 
TODO:
def env_reset()         # reset the environmrnt
def env_animation()     # show animation
def env_step()          # one step  
"""

# Reset the environment
# Train single agent and by default train the first agent
# Maybe changed later
def env_reset(filename, agent):
    #----------------------------------------------#

    # TODO:
    # Train each agent

    # ---------------------------------------------#
    my_map, starts, goals = import_mapf_instance(filename)

    # treat other agents as block, true indicates block, exclude itself as block
    for x,y in starts[1:]:
        my_map[x][y] = True

    # True as 1 and False as 0
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            my_map[i][j] = 0 if not my_map[i][j] else 1

    start, goal = starts[agent.index], goals[agent.index]
    return my_map, start, goal

# Get reward and next state
def Nextstep(a):
    pass

# 定义Net类 (定义网络)
class Net(nn.Module):
    def __init__(self):                                                         # 定义Net的一系列属性
        # nn.Module的子类函数必须在构造函数中执行父类的构造函数
        super(Net, self).__init__()                                             # 等价与nn.Module.__init__()

        self.fc1 = nn.Linear(N_STATES, 50)                                      # 设置第一个全连接层(输入层到隐藏层): 状态数个神经元到50个神经元
        self.fc1.weight.data.normal_(0, 0.1)                                    # 权重初始化 (均值为0，方差为0.1的正态分布)
        self.fc2 = nn.Linear(50, N_ACTIONS)                                     # 设置第二个全连接层(隐藏层到输出层): 50个神经元到动作数个神经元
        self.fc2.weight.data.normal_(0, 0.1)                                    # 权重初始化 (均值为0，方差为0.1的正态分布)

    def forward(self, x):                                                       # 定义forward函数 (x为状态)
        x = F.relu(self.fc1(x))                                                 # 连接输入层到隐藏层，且使用激励函数ReLU来处理经过隐藏层后的值
        actions_value = self.fc2(x)                                             # 连接隐藏层到输出层，获得最终的输出值 (即动作值)
        return actions_value                                                    # 返回动作值


# Define DQN (define two network)
class DQN(object):
    def __init__(self):                                                         # 定义DQN的一系列属性
        self.eval_net, self.target_net = Net(), Net()                           # 利用Net创建两个神经网络: 评估网络和目标网络
        self.learn_step_counter = 0                                             # for target updating
        self.memory_counter = 0                                                 # for storing memory
        self.memory = np.zeros((MEMORY_CAPACITY, N_STATES * 2 + 2))             # 初始化记忆库，一行代表一个transition
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=LR)    # 使用Adam优化器 (输入为评估网络的参数和学习率)
        self.loss_func = nn.MSELoss()                                           # 使用均方损失函数 (loss(xi, yi)=(xi-yi)^2)

    def choose_action(self, state):                                             # 定义动作选择函数 (x为状态)

        # ----------------------------------------------#

        # TODO:
        # should we wrap up all info about the current state of agent into a feature vector? should be a matrix
        # all info contains current position, map info, other agents info, treat other agent as block? Yes
        # the action have only 4 possible movement, should we use softmax output a distribution?   Yes

        cur_map, cur_pos = state
        cur_map[cur_pos[0]][cur_pos[1]] = 2                                     # 2 indicates the current position in the map
        x = torch.FloatTensor(cur_map).view(-1, 1)                              # vectorized the matrx

        # ---------------------------------------------#

        if np.random.uniform() < EPSILON:                                       # choose network best action or random action base on epsilon
            actions_value = self.eval_net.forward(x)                            # get action value
            action_index = int(torch.argmax(actions_value))                     # get the index of most possible action
        else:                                                                   # random action
            action_index = np.random.randint(0, N_ACTIONS)                      # random action of the five action

        if action_index == 0:                                                   # go up
            action = [1, 0]
        elif action_index == 1:                                                 # go down
            action = [-1, 0]
        if action_index == 2:                                                   # go right
            action = [0, 1]
        if action_index == 3:                                                   # go left
            action = [0, -1]
        else:                                                                   # stay
            action = [0, 0]

        return action

    def store_transition(self, s, a, r, s_):                                    # 定义记忆存储函数 (这里输入为一个transition)
        transition = np.hstack((s, [a, r], s_))                                 # 在水平方向上拼接数组
        # 如果记忆库满了，便覆盖旧的数据
        index = self.memory_counter % MEMORY_CAPACITY                           # 获取transition要置入的行数
        self.memory[index, :] = transition                                      # 置入transition
        self.memory_counter += 1                                                # memory_counter自加1

    def learn(self):                                                            # 定义学习函数(记忆库已满后便开始学习)
        # 目标网络参数更新
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:                  # 一开始触发，然后每100步触发
            self.target_net.load_state_dict(self.eval_net.state_dict())         # 将评估网络的参数赋给目标网络
        self.learn_step_counter += 1                                            # 学习步数自加1

        # 抽取记忆库中的批数据
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)            # 在[0, 2000)内随机抽取32个数，可能会重复
        b_memory = self.memory[sample_index, :]                                 # 抽取32个索引对应的32个transition，存入b_memory
        b_s = torch.FloatTensor(b_memory[:, :N_STATES])
        # 将32个s抽出，转为32-bit floating point形式，并存储到b_s中，b_s为32行4列
        b_a = torch.LongTensor(b_memory[:, N_STATES:N_STATES+1].astype(int))
        # 将32个a抽出，转为64-bit integer (signed)形式，并存储到b_a中 (之所以为LongTensor类型，是为了方便后面torch.gather的使用)，b_a为32行1列
        b_r = torch.FloatTensor(b_memory[:, N_STATES+1:N_STATES+2])
        # 将32个r抽出，转为32-bit floating point形式，并存储到b_s中，b_r为32行1列
        b_s_ = torch.FloatTensor(b_memory[:, -N_STATES:])
        # 将32个s_抽出，转为32-bit floating point形式，并存储到b_s中，b_s_为32行4列

        # 获取32个transition的评估值和目标值，并利用损失函数和优化器进行评估网络参数更新
        q_eval = self.eval_net(b_s).gather(1, b_a)
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


def train(filename, agent):
    dqn = DQN()                                                         # initialize a dqn
    my_map, start, goal = env_reset(filename, agent)                    # initial state: map, starts, goals
    N_STATES = len(my_map) * len(my_map[0])

    for i in range(400):                                                # 400 episode loop
        print('<<<<<<<<<Episode: %s' % i)
        cur_map, cur_pos, cur_goal = my_map, start, goal                # reset environment
        episode_reward_sum = 0                                          # initialize current episode total reward

        while True:                                                     # start an episode (each loop indicates a step)
            s = [cur_map, cur_pos]                                      # state consists of current map(block,other agents as block) and position
            a = dqn.choose_action(s)                                    # input the current state and choose an action
            # reward: agent achieve goal: +0, collision: -inf, gonext: -1
            s_, r, done = agent.nextStep(a)                             # conduct action and require feedback, how to get reward r?

            # modify reward (it could be original reward，modify reward for better train speed)
            # x, x_dot, theta, theta_dot = s_
            # r1 = (env.x_threshold - abs(x)) / env.x_threshold - 0.8
            # r2 = (env.theta_threshold_radians - abs(theta)) / env.theta_threshold_radians - 0.5
            # new_r = r1 + r2

            dqn.store_transition(s, a, new_r, s_)                 # 存储样本
            episode_reward_sum += new_r                           # 逐步加上一个episode内每个step的reward

            s = s_                                                # 更新状态

            if dqn.memory_counter > MEMORY_CAPACITY:              # 如果累计的transition数量超过了记忆库的固定容量2000
                # 开始学习 (抽取记忆，即32个transition，并对评估网络参数进行更新，并在开始学习后每隔100次将评估网络的参数赋给目标网络)
                dqn.learn()

            if done:       # 如果done为True
                # round()方法返回episode_reward_sum的小数点四舍五入到2个数字
                print('episode%s---reward_sum: %s' % (i, round(episode_reward_sum, 2)))
                break                                             # 该episode结束


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training the MAPF solver...')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    args = parser.parse_args()
    filename = args.instance

    index = 0;
    agent = Agent()

    #train(filename)