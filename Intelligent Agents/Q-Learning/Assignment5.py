import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import numpy as np
import json
import random


random.seed(42)

# Updated GridWorld class that can load saved environments
class LoadableGridWorld:
    def __init__(self, environment_file=None):
        if environment_file:
            with open(environment_file, 'r') as f:
                env_data = json.load(f)
            
            self.grid_size = env_data['grid_size']
            self.obstacles = [tuple(obs) for obs in env_data['obstacles']]
            self.start_pos = tuple(env_data['start_pos']) if env_data['start_pos'] else None
            self.goal_pos = tuple(env_data['goal_pos']) if env_data['goal_pos'] else None
            self.trajectory = [tuple(pos) for pos in env_data['trajectory']]
        else:
            self.grid_size = 8
            self.obstacles = []
            self.start_pos = (7, 0)
            self.goal_pos = (0, 7)
            self.trajectory = []
        
        self.reset()
    
    def reset(self):
        self.robot_learner = self.start_pos if self.start_pos else (7, 0)
        self.robot_fixed = self.goal_pos if self.goal_pos else (0, 7)
        self.target = self.goal_pos if self.goal_pos else (0, 7)
        self.done = False
        self.learned_path = []
        self.trajectory_index = 0
        return self.get_state()
    
    def get_state(self):
        return self.robot_learner + self.robot_fixed
    
    def is_valid_position(self, position):
        if self.trajectory_index < len(self.trajectory):
            self.robot_fixed = self.trajectory[self.trajectory_index]
        if self.robot_fixed == position:
            return False
        x, y = position
        return (0 <= x < self.grid_size and 
                0 <= y < self.grid_size and 
                position not in self.obstacles)
    
    def step(self, action):
        if self.done:
            self.done = True
            return self.get_state(), 0, True
        
        # Action mapping: 0=up, 1=right, 2=down, 3=left
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1), (0, 0)]
        dx, dy = directions[action]
        
        # Calculate new position for learning robot
        new_x = self.robot_learner[0] + dx
        new_y = self.robot_learner[1] + dy
        new_position = (new_x, new_y)
        
        # Check if valid move
        if self.is_valid_position(new_position) and self.is_valid_position(self.robot_learner):
            self.robot_learner = new_position
            self.learned_path.append(new_position)
            
            # Move fixed robot along trajectory if available
            if self.trajectory and self.trajectory_index < len(self.trajectory):
                self.robot_fixed = self.trajectory[self.trajectory_index]
                self.trajectory_index += 1
            
            reward = -1  # Small penalty for each step
         
            
            # Check if reached target
            if self.robot_learner == self.target:
                reward = 100
                self.done = True
        else:
            reward = -10  # Larger penalty for invalid moves
            self.done = True
        
        return self.get_state(), reward, self.done
    
    def render(self, title='Grid World', display_path=False):
        plt.clf()
        
        # Create white background
        plt.figure(1, facecolor='white', figsize=(10, 10))
        ax = plt.gca()
        ax.set_facecolor('white')
        
        # Draw obstacles
        for obs in self.obstacles:
            rect = Rectangle((obs[1]-0.5, obs[0]-0.5), 1, 1, color='gray', alpha=0.8)
            ax.add_patch(rect)
        
        # Draw trajectory
        if len(self.trajectory) > 1:
            traj_x = [pos[1] for pos in self.trajectory]
            traj_y = [pos[0] for pos in self.trajectory]
            plt.plot(traj_x, traj_y, 'g-', alpha=0.8, linewidth=3)
        
        # Draw learned path if requested
        if display_path and len(self.learned_path) > 1:
            path_x = [pos[1] for pos in self.learned_path]
            path_y = [pos[0] for pos in self.learned_path]
            plt.plot(path_x, path_y, 'b-', alpha=0.5, linewidth=2)
        
        # Plot learning robot (blue circle)
        plt.scatter(self.robot_learner[1], self.robot_learner[0], 
                   c='blue', s=300, marker='o', edgecolors='black', linewidth=2)
        
        # Plot fixed robot (green square)
        plt.scatter(self.robot_fixed[1], self.robot_fixed[0], 
                   c='green', s=300, marker='s', edgecolors='black', linewidth=2)
        
        # Plot target (red star)
        if self.target:
            plt.scatter(self.target[1], self.target[0], 
                       c='red', s=400, marker='*', edgecolors='black', linewidth=1)
        
        plt.xlim(-0.5, self.grid_size-0.5)
        plt.ylim(self.grid_size-0.5, -0.5)
        plt.xticks(range(self.grid_size))
        plt.yticks(range(self.grid_size))
        plt.grid(True, alpha=0.3)
        plt.title(title)
        plt.tight_layout()


# Implement Q-learning agent
class QLearningAgent:
    def __init__(self, n_actions=5, learning_rate=0.1, discount_factor=0.95, epsilon=0.1):
        self.n_actions = n_actions
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        self.q_table = {}

    def get_q_value(self, state, action):
        if state not in self.q_table:
            self.q_table[state] = [0.0] * self.n_actions
        return self.q_table[state][action]

    def update_q_value(self, state, action, reward, next_state):
        if state not in self.q_table:
            self.q_table[state] = [0.0] * self.n_actions
        if next_state not in self.q_table:
            self.q_table[next_state] = [0.0] * self.n_actions

        best_next_q = max(self.q_table[next_state])
        current_q = self.q_table[state][action]
        updated_q = current_q + self.learning_rate * (reward + self.discount_factor * best_next_q - current_q)
        self.q_table[state][action] = updated_q

    def get_action(self, state):
        if state not in self.q_table:
            self.q_table[state] = [0.0] * self.n_actions

        if random.random() < self.epsilon:
            return random.randint(0, self.n_actions - 1)
        else:
            return int(np.argmax(self.q_table[state]))

def train(episodes=5000, epsilon_decay=0.99):
    env = LoadableGridWorld('grid_world_environment.json')
    agent = QLearningAgent()
    rewards_history = []

    for episode in range(episodes):
        state = env.reset()
        total_reward = 0
        done = False
        step_count = 0
        max_steps = 100

        while not done and step_count < max_steps:
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            agent.update_q_value(state, action, reward, next_state)
            state = next_state
            total_reward += reward
            step_count += 1

        rewards_history.append(total_reward)
        agent.epsilon *= epsilon_decay  # Decay exploration rate
        agent.epsilon = max(0.01, agent.epsilon)  # Clamp minimum epsilon

        if (episode + 1) % 500 == 0:
            print(f"Episode {episode+1}: Total reward = {total_reward:.2f}, Epsilon = {agent.epsilon:.4f}")

    return agent, rewards_history


def animate_final_solution(agent):
    env = LoadableGridWorld('grid_world_environment.json')
    state = env.reset()
    done = False
    steps = 0
    
    plt.figure(1, figsize=(10, 10))
    
    # Animate both robots following their paths
    while not done and steps < 500:
        action = agent.get_action(state)
        next_state, _, done = env.step(action)
        state = next_state
        steps += 1
        
        env.render(title=f'Step {steps}: Maze Navigation', display_path=True)
        plt.pause(0.1)
        plt.draw()
    
    plt.show(block=True)
    print(f"Animation completed in {steps} steps")

# Train the agent
print("Training Q-learning agent...")
trained_agent, rewards = train(episodes=3000)

print("\nAnimating final solution...")
animate_final_solution(trained_agent)

# Plot training progress
moving_average_rewards = rewards.copy()
N = len(moving_average_rewards)
alpha = 0.99
for i in range(1, N):
    moving_average_rewards[i] = alpha * moving_average_rewards[i - 1] + (1 - alpha) * moving_average_rewards[i]

plt.figure(figsize=(10, 5))
plt.plot(rewards, color='blue', label='Episode Reward', alpha=0.4)
plt.plot(moving_average_rewards, color='orange', label='Moving Average')
plt.title('Training Progress: Rewards per Episode')
plt.xlabel('Episode')
plt.ylabel('Total Reward')
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig('training_progress.png')
plt.show()