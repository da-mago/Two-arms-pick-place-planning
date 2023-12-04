import numpy as np
import time
import platform
from subprocess import Popen, PIPE
import ctypes
from numpy.ctypeslib import ndpointer



class mdp_solver():
    ''' Find an optimal solution for a tabular MDP.
        Note: a MDP can have more than one optimal solution

        Select algorithm to apply:
        - 0: Value Iteration
        - 1: BFS
        - 2: Dijkstra
    '''

    # Algorithms
    ALG_VALUE_ITERATION = 0
    ALG_BFS = 1
    ALG_DIJKSTRA = 2

    def __init__(self, MDP, init_state=0):
        self.MDP = MDP
        self.init_state = init_state

# CMG comentado hasta generar la lib
#        # Load BFS C implementation
#        if platform.system() == "Windows":
#            filename = "./c_bfs.dll"
#        else:
#            filename = "./c_bfs.so"
#
#        lib = ctypes.cdll.LoadLibrary(filename)
#
#        lib.BFS.restype = None
#        lib.BFS.argtypes = [ ctypes.c_size_t,  # init_state
#                             ctypes.c_size_t,  # n_states
#                             ctypes.c_size_t,  # n_actions
#                             ndpointer(ctypes.c_uint32, flags="C_CONTIGUOUS"), # states
#                             ndpointer(ctypes.c_int16, flags="C_CONTIGUOUS"),  # rewards
#                             ndpointer(ctypes.c_uint32, flags="C_CONTIGUOUS"), # path
#                             ndpointer(ctypes.c_uint32, flags="C_CONTIGUOUS"), # path_len
#                             ndpointer(ctypes.c_uint16, flags="C_CONTIGUOUS")] # actions
#
#        self.C_BFS = lib.BFS

    def solve(self, algorithm=0):

        # Value Iteration
        if algorithm == mdp_solver.ALG_VALUE_ITERATION:
            path = None
            #policy = self._matrixValueIteration(discount_factor = 0.95)
            policy = self._matrixValueIteration(discount_factor = 1)
        ## BFS (graph search)
        elif algorithm == mdp_solver.ALG_BFS:
            # to keep the same interface, fake a policy object
            path = self._BFS()
            policy = None
        ## Dijkstra (graph search)
        #elif algorithm == mdp_solver.ALG_DIJKSTRA:

        #    path = Dijkstra(MDP)
        #    policy = None
        else:
            print("Unknown algorithm")
            import sys
            sys.exit()
            
        return policy, path

    def _matrixValueIteration(self, theta=0.0001, discount_factor=1.0):
        ''' Value Iteration.

            Do not iterate for each state-action pair at python level, 
            model it as matrices and let numpy do the magic
        '''
    
        print("Solver: Value Iteration")

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
    
        print("Solver: BFS")

        states     = self.MDP[0]
        rewards    = self.MDP[1]

        # Call BFS from C shared library
        if True:
            path       = np.ones((100,), dtype=np.uint32)
            path_len   = np.ones((1,), dtype=np.uint32)
            actions    = np.ones((100,), dtype=np.uint16)
            n_states   = len(states)
            n_actions  = len(states[0])
            self.C_BFS(self.init_state, n_states, n_actions, states, rewards, path, path_len, actions)
            path_len = int(path_len)
            path = path[:path_len][::-1]
            actions = actions[:path_len][::-1]

            pathNactions = [path, actions]

            return pathNactions

        # Python BFS version
        else:
            n_states = len(states)
            frontier = [0 for _ in range(n_states)]
            visited  = [0 for _ in range(n_states)]
            frontier_fifo = [init_state]
            linkedList = {}
            count= 0
            while len(frontier_fifo)>0:
                count += 1
                state = frontier_fifo[0]
                frontier_fifo.pop(0)
                frontier[state] = 0
                visited[state] = 1
    
                for action in range(len(states[0])):
                    r = rewards[state,action] 
                    if r < (-1):
                        # discard it (bad action)
                        continue
                    elif r > 50:
                        # Done
                        next_state = states[state,action]
                        linkedList[next_state] = state
                        path = [next_state]
                        while True:
                            print("{} ".format(next_state))
                            if next_state not in linkedList:
                                break
                            next_state = linkedList[next_state]
    
                            if next_state == init_state:
                                # not including initial state
                                print("{} init state\n".format(next_state))
                                break
                            path.insert(0, next_state)
                        return path
    
                    next_state = states[state,action]
                    if visited[next_state] == 0 and frontier[next_state] == 0:
                        frontier[next_state] = 1
                        frontier_fifo.append(next_state)
                        linkedList[next_state] = state
    
            print("  Path not found!!!", count)
            return []

if __name__ == "__main__":

    MDP = [ np.random.randint(4, size=(5,3)), np.random.randint(100, size=(5,3)) ]
    solver = mdp_solver(MDP) 
    policy = solver.solve()

    print(policy)


