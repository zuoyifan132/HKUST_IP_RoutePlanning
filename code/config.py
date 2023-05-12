from itertools import product


# hyper parameters
BATCH_SIZE = 32                                 # batch size
LR = 1e-3                                       # learning rate
EPSILON = 0.1                                   # initial greedy policy
GAMMA = 0.9                                     # reward discount: close to 0 weights more on immediate and close 1 weight more on future reward
TARGET_REPLACE_ITER = 100                       # target update frequency
MEMORY_CAPACITY = 2000                          # memory capacity
NUM_ACTION = 5                                  # there are 5 action: left, right, up, down, stay and 4 agents
NUM_AGENT = 2
N_STATES = 100                                  # an agent state include the agent position and map state
EPISODE = 1000                                   # episode number

# map
map_height = 10
map_width = 10

# map encoding: 1: free grid, 2: block, 3: other agent, 100: self agent, goal: 5
# self agent should be 100 number to make network treat self agent position significantly
free_grid = 1
block = 2
self_agent = 100
goal = 5

# action map: 0: up, 1: down, 2: right, 3: left, 4: stay
actions = [(-1, 0), (1, 0), (0, 1), (0, -1), (0, 0)]
agents_Actions = list(product(actions, repeat=NUM_AGENT))
print(agents_Actions)
Action_map = {i: agents_Actions[i] for i in range(len(agents_Actions))}

# reward
collide_with_block = float(-5)
collide_with_agent = float(-5)
reach_goal = float(100)
doesnt_collide = float(-0.5)
heuristic_fine_tune_weight = 5