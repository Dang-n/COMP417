
import numpy as np


class RL_controller:
    def __init__(self, args):
        self.gamma = args.gamma
        self.lr = args.lr
        self.Q_value = np.zeros((args.theta_discrete_steps, args.theta_dot_discrete_steps, 3)) # state-action values
        self.V_values = np.zeros((args.theta_discrete_steps, args.theta_dot_discrete_steps)) # state values
        self.prev_a = 0 # previous action
        # Use a previous_state = None to detect the beginning of the new round e.g. if not(self.prev_s is None): ...
        self.prev_s = None # Previous state

    def reset(self):
        #You need to reset sth
        self.prev_a = 0
        self.prev_s = None

    def get_action(self, state, image_state, random_controller=False, episode=0):

        terminal, timestep, theta, theta_dot, reward = state
        if random_controller:
            action = np.random.randint(0, 3) # you have three possible actions (0,1,2)
        else:
            #Randomized action 
            if np.random.rand() > 0.8:
                action = np.random.randint(0, 3)
            # use Q values to take the best action at each state
            else:
                possibleActions = [self.Q_value[theta, theta_dot, 0],self.Q_value[theta, theta_dot, 1],self.Q_value[theta, theta_dot, 2]]
                max_val = max(possibleActions)
                max_ac = possibleActions.index(max_val)
                action = max_ac

        if not(self.prev_s is None or self.prev_s == [theta, theta_dot]):
            possibleActions = [self.Q_value[theta, theta_dot, 0],self.Q_value[theta, theta_dot, 1],self.Q_value[theta, theta_dot, 2]]
            max_val = max(possibleActions)
            max_ac = possibleActions.index(max_val)
            self.Q_value[self.prev_s[0], self.prev_s[1], self.prev_a] = self.Q_value[self.prev_s[0], self.prev_s[1], self.prev_a] + self.lr * ( reward + self.gamma * (self.Q_value[theta, theta_dot, max_ac]) - self.Q_value[self.prev_s[0], self.prev_s[1], self.prev_a])
            
            self.V_values[self.prev_s[0], self.prev_s[1]] = reward + self.gamma * self.V_values[theta, theta_dot] 


        #############################################################
        self.prev_s = [theta, theta_dot]
        self.prev_a = action
        return action

 