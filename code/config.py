
# hyper parameters
BATCH_SIZE = 32                                 # batch size
LR = 1e-3                                       # learning rate
EPSILON = 0.1                                   # initial greedy policy
GAMMA = 0.9                                     # reward discount: close to 0 weights more on immediate and close 1 weight more on future reward
TARGET_REPLACE_ITER = 100                       # target update frequency
MEMORY_CAPACITY = 2000                          # memory capacity
N_ACTIONS = 5                                   # there are 5 action: left, right, up, down, stay
N_STATES = 100                                  # an agent state include the agent position and map state
EPISODE = 2000                                  # episode number

# map encoding: 1: free grid, 2: block, 3: other agent, 100: self agent, goal: 5
# self agent should be 100 number to make network treat self agent position significantly
free_grid = 1
block = 2
other_agent = 3
self_agent = 100
goal = 5

# action map: 0: up, 1: down, 2: right, 3: left, 4: stay
Action_map = {0: [-1, 0], 1: [1, 0], 2: [0, 1], 3: [0, -1], 4: [0, 0]}
Index_action_map = {(-1, 0): 0, (1, 0): 1, (0, 1): 2, (0, -1): 3, (0, 0): 4}

# reward
collide_with_block = float(-5)
collide_with_agent = float(-5)
reach_goal = float(100)
doesnt_collide = float(-0.5)