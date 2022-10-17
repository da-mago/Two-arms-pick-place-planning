import numpy as np

class mdp_solver():
    ''' Find an optimal solution for a tabular MDP.
        Note: a MDP can have more than one optimal solution

        Select algorithm to apply:
        - 0: Value Iteration
        - 1: BFS
        - 2: Dijkstra
    '''

    def __init__(self, MDP):
        self.MDP = MDP

    def solve(self, algorithm=0):

        # Value Iteration
        if algorithm == 0:
            #policy = self._npVI(discount_factor = 0.95)
            policy = self._npVI(discount_factor = 1)
        ## BFS (graph search)
        #elif algorithm == 1:
        #    path = BFS(MDP)
        ## Dijkstra (graph search)
        #elif algorithm == 2:
        #    path = Dijkstra(MDP)


        return policy

    def _npVI(self, theta=0.0001, discount_factor=1.0):
        ''' Value Iteration.

            Note: npVI stands for NumPy Value Iteration.
            Do not iterate for each state at python level, use numpy power
        '''
    
        states, rewards = self.MDP
        V = np.zeros(len(states), dtype=np.float32)
    
        self.rewards = rewards
        i=0
        delta_old = 0
        while True:
            Vk = np.max(rewards + discount_factor*V[states], axis=1)
            delta = np.max(np.abs(Vk-V))
            #print(delta, np.argmax(np.abs(Vk-V)))
            self.V = V
            self.Vk = Vk
            V = Vk
            #if delta < theta or delta == 1:
            #if delta < theta or delta == 20:
            if delta < theta:
                break

            ## Force end (states island never converge with discount_factor=1 )
            if delta == delta_old:
                i+=1
                if i>80:
                    #print('BREAK SOLVE', delta)
                    break
            else:
                delta_old = delta
                i=0
    
        VA = rewards + discount_factor*V[states]

        NUM_ACTION = 29
        ACTION_STAY = 28
        y = ACTION_STAY
        left_arm  = [NUM_ACTION*x + y for x in range(ACTION_STAY)]
        x = ACTION_STAY
        right_arm = [NUM_ACTION*x + y for y in range(ACTION_STAY)]
        two_arm   = [NUM_ACTION*x + y for x in range(ACTION_STAY) for y in range(ACTION_STAY)]
        all_arm = np.array(left_arm + right_arm + two_arm)
        VA2 = np.zeros_like(VA)
        #print(left_arm, right_arm, two_arm, all_arm, VA2.shape)
        VA2[:, 0:len(left_arm)] = VA[:, left_arm]
        VA2[:, len(left_arm):len(left_arm + right_arm)] = VA[:, right_arm]
        VA2[:, len(left_arm + right_arm):]  = VA[:, two_arm]

        policy = all_arm[np.argmax(VA2, axis=1)]
#        policy = np.argmax(VA, axis=1)

        #print('SOLVED')
    
        return policy


if __name__ == "__main__":

    MDP = [ np.random.randint(4, size=(5,3)), np.random.randint(100, size=(5,3)) ]
    solver = mdp_solver(MDP) 
    policy = solver.solve()

    print(policy)


