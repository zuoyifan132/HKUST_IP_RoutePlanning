import gym
import torch
import torch.optim as optim
import torch.nn.functional as F

from simulator import Simulator
from agent import DQNAgent

# number of training
episode = 10

# Define the reinforcement learning agent
agent = DQNAgent(state_size=10, action_size=4)

# Define the environment
env = Simulator()

# Define the optimizer and loss function
optimizer = optim.Adam(agent.parameters(), lr=0.001)
mse_loss = F.mse_loss

# Train the agent
for episode in range(1000):
    state = env.reset()
    total_reward = 0
    done = False
    while not done:
        # Choose action using epsilon-greedy policy
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        total_reward += reward

        # Save experience and update network
        agent.memorize(state, action, reward, next_state, done)
        loss = agent.learn(optimizer, mse_loss)

        # Update state
        state = next_state

    # Print results
    print(f"Episode: {episode}, Total reward: {total_reward}")

# Evaluate the agent
state = env.reset()
done = False
while not done:
    action = agent.act(state, greedy=True)
    state, reward, done, _ = env.step(action)

env.close()
