import numpy as np
from reinforcement_lib_v2 import DQN, Memory
import gym, sys



def log(file_name, text=None, act='a'):
    """
    Function to save log file
    """

    fout = open(file_name + ".csv", act)
    if text is not None:
        fout.write(text+'\n')
    fout.close()


# simulation control variables
episode = 100

# creating evironment
env = gym.make("LunarLander-v2")
num_states = 8
num_actions = 1
network_name = sys.argv[1] 
#input('Type the model name:') #'Lunar_01'

"""
Permissible Actions
0- Do nothing
1- Fire left engine
2- Fire down engine
3- Fire right engine
"""

actions = [0,1,2,3]

# loading network
network = DQN(num_states,num_actions, model_name=network_name, load_model=True)

# system counter
n = 0
curve = np.zeros(episode)

print('Model name:',network_name,'\nEnv States:',num_states,'\nEnv Actions:',num_actions)
print('System read!\nStarting Rover Control...')

# creating training loop
for i in range(episode):
    observation = env.reset()

    done = False

    while not done:

        n+=1

        # render environment
        if True:#i%episode == 99:
            env.render()

        # selection action using network
        action = actions[np.argmax(network(observation,actions))]

        # performing action
        observation, reward, done, info = env.step(action)
        curve[i] += reward
    print(str(observation))

    log(network_name,str(observation))
    print(curve[i])
print('Training done!')
