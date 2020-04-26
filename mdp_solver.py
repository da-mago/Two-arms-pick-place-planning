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
            policy = self._npVI(discount_factor = 0.95)
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
        V = np.zeros(len(states), dtype=np.float)
    
        while True:
            Vk = np.max(rewards + discount_factor*V[states], axis=1)
            delta = np.max(np.abs(Vk-V))
            V = Vk
            if delta < theta:
                break
    
        policy = np.argmax(rewards + discount_factor*V[states], axis=1)
    
        return policy


if __name__ == "__main__":

    MDP = [ np.random.randint(4, size=(5,3)), np.random.randint(100, size=(5,3)) ]
    solver = mdp_solver(MDP) 
    policy = solver.solve()

    print(policy)


