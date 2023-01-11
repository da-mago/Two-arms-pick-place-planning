import numpy as np

class mdp_solver():
    ''' Find an optimal solution for a tabular MDP.
        Note: a MDP can have more than one optimal solution

        Select algorithm to apply:
        - 0: Value Iteration
        - 1: BFS
        - 2: Dijkstra
    '''

    def __init__(self, MDP, init_state=0):
        self.MDP = MDP
        self.init_state = init_state

    def solve(self, algorithm=0):

        # Value Iteration
        if algorithm == 0:
            #policy = self._npVI(discount_factor = 0.95)
            policy = self._npVI(discount_factor = 1)
        ## BFS (graph search)
        elif algorithm == 1:
            # to keep the same interface, fake a policy object
            policy = self._BFS()
        ## Dijkstra (graph search)
        #elif algorithm == 2:
        #    path = Dijkstra(MDP)
        else:
            print("Unknown algorithm")
            import sys
            sys.exit()
            
        return policy

    def _npVI(self, theta=0.0001, discount_factor=1.0):
        ''' Value Iteration.

            Note: npVI stands for NumPy Value Iteration.
            Do not iterate for each state at python level, use numpy power
        '''
    
        print("Solving MDP by Value Iteration algorithm")

        states, rewards  = self.MDP
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
    
        # Optimal Value Function
        VA = rewards + discount_factor*V[states]

        # Compute policy (best action for each state)
        policy = np.argmax(VA, axis=1)
    
        return policy


    def _BFS(self):
        ''' Bread First Serch algorithm '''
    
        print("Solving MDP by BFS algorithm")

        states, rewards  = self.MDP
        frontier = [self.init_state]
        visited = []
        linkedList = {}
        while len(frontier)>0:
            state = frontier[0]
            frontier.pop(0)
            visited.append(state)

            for action in range(len(states[0])):
                r = rewards[state, action] 
                if r < (-1):
                    # discard it (bad action)
                    continue
                elif r > 50:
                    # Done
                    next_state = states[state, action]
                    linkedList[next_state] = state
                    path = [next_state]
                    while True:
                        if next_state not in linkedList:
                            break
                        next_state = linkedList[next_state]
                        path.insert(0, next_state)
                    return path

                next_state = states[state, action]
                if next_state not in visited and next_state not in frontier:
                    frontier.append(next_state)
                    linkedList[next_state] = state


if __name__ == "__main__":

    MDP = [ np.random.randint(4, size=(5,3)), np.random.randint(100, size=(5,3)) ]
    solver = mdp_solver(MDP) 
    policy = solver.solve()

    print(policy)


