###############################################################################
#
# Convert any deterministic RL environment to a tabular MDP.
# Note: that speeds up the computation when searching for an optimal solution.
#
# Moreover, a deterministic MDP can be reduced to a graph. That allows you to 
# apply both graph search and RL algorithms to it.
#

from env_pickplace import env_pickplace
#from robot_2A2L import Robot_2A2L
from robot_YuMi import Robot_YuMi
import numpy as np
import pickle

class mdp_generator(env_pickplace):
    def __init__(self, robot, pieces):
        super().__init__(robot, pieces)

    def load(self, filename):
        ''' Load MDP from file '''
        try:
            with open(filename, 'rb') as f:
                self.MDP = pickle.load(f)
            return True
        except:
            return False 

    def save(self, filename):
        ''' Save MDP to file '''
        with open(filename, 'wb') as f:
            pickle.dump(self.MDP, f)

    def generate(self):
        ''' Generate MDP based on a specific environment

            MDP generation is split in two phases:
            - MDP generation assuming pieces configuration is unknown. This step
              can be done offline and is valid for any use case.
              - use generate() method.
            - Update MDP for a specific use case (pieces set configuration).
              This step is done online once you know which use case you need to
              solve.
              - use updateMDP() method

            The idea behind these two-step generation process is to reduce the 
            online computation.
        '''

        print("Filtering valid states ...")
        print(self.nS)

        # Filter out invalid states
        valid_states = [s for s in range(self.nS) if self._isStateValid(s)]
        valid_nS = len(valid_states)
        print(valid_nS)

        states_idx = {}
        for i, item in enumerate(valid_states):
            states_idx[item] = i

        print("Computing MDP ...")
        # Note: MDP is very large. Use numpy
        mdp_s = np.zeros((valid_nS, self.nA), dtype=np.int32) 
        mdp_r = np.zeros((valid_nS, self.nA), dtype=np.int8)
        mdp_v = np.array(valid_states)
        mdp_i = {x:i for i,x in enumerate(mdp_v)}
        for i,s in enumerate(valid_states):
            for a in range(self.nA):
               self.reset(s)
               next_state, reward, done, info = self._step(a, mode=1)
               mdp_s[i][a] = states_idx[next_state]
               mdp_r[i][a] = reward
            if (i % (valid_nS//10)) == 0:
                print(10*i//(valid_nS//10),'%')

        # State space layers partition
        # TODO: prioritized_piecesMap must be generic fr n-pieces
        prioritized_piecesMap = [0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0x7, 0xb, 0xd, 0xe, 0xf]
        prioritized_status    = [[0,0],[1,0],[2,0],[3,0],[4,0],[0,1],[0,2],[0,3],[0,4],[1,2],[1,3],[1,4],[2,1],[2,3],[2,4],[3,1],[3,2],[3,4],[4,1],[4,2],[4,3]]
        piecesMap_len = len(prioritized_piecesMap)
        status_len    = len(prioritized_status)
        mdp_l = [[] for _ in range(piecesMap_len * status_len)]
        for i, state in enumerate(mdp_v):
            armsGridPos, armsStatus, piecesMap = self._ext2intState(state)
            idx = prioritized_status.index(armsStatus) + status_len*prioritized_piecesMap[piecesMap]
            mdp_l[idx].append(i)

        self.MDP = [mdp_s, mdp_r, mdp_v, mdp_i, mdp_l] # next_state, reward, state redution mapping, inverse state reduction mapping, layered partitions

    def update(self):
        ''' Update MDP for a specific configuration of pieces '''
    
        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
    
        p_ini = [pos for pos in self.piecesLocation['start']]
        p_end = [pos for pos in self.piecesLocation['end']  ]
    
        states_idx = self.MDP[3]
        for arm in range(2):
            for i, (pos_ini,pos_end) in enumerate(zip(p_ini, p_end)):
                for x in range(self.M):
                    for y in range(self.N):
                        for status_2 in range(self.K+1):
                            for bitmap in range(2**(self.K)):
                                if bitmap & (1<<i):
                                    # pick
                                    pos = pos_ini
                                    status = 0
                                    next_status = i+1
                                    tmp_bitmap = bitmap & ~(1<<i)
                                    a = 4
                                    # place
                                else:
                                    pos = pos_end
                                    status = i+1
                                    next_status = 0
                                    tmp_bitmap = bitmap
                                    a = 5
    
                                if arm == 0:
                                    state = self._int2extState([pos, [x,y]], [status, status_2], bitmap)
                                else:
                                    state = self._int2extState([[x,y], pos], [status_2, status], bitmap)
    
                                # Process only valid states
                                if state in states_idx:
                                    
                                    idx = states_idx[state]
                                    # Avoid checks (it was done in phase 1, -30 means all checks were passed)
                                    for action in range(7):
                                        if arm == 0: a1 = a;      a2 = action
                                        else:        a1 = action; a2 = a
    
                                        next_bitmap = tmp_bitmap
                                        if self.MDP[1][idx][a1*7+a2] == -30: # pick/any
                                            # remove mark
                                            self.MDP[1][idx][a1*7+a2] = -20
    
                                            next_status_2 = status_2
                                            if action < 4: 
                                                pos2 = list([x,y] + offset_a[action])
                                            else:
                                                pos2 = [x,y]
                                                if action == 4:
                                                    if (status_2 == 0) and ([x,y] in p_ini):
                                                        tmp = p_ini.index([x,y])
                                                        if bitmap & (1 << tmp):
                                                            next_status_2 = tmp+1
                                                            next_bitmap &= ~(1<<tmp)
                                                        else:
                                                            continue
                                                    else:
                                                        continue
                                                elif action == 5:
                                                    if (status_2 > 0) and ([x,y] in p_end):
                                                        tmp = p_end.index([x,y])
                                                        if (bitmap & (1 << tmp)) == 0 and status_2 == tmp+1:
                                                            next_status_2 = 0
                                                        else:
                                                            continue
                                                    else:
                                                        continue
    
                                            # update next_state and reward
                                            if arm == 0: state =  self._int2extState([pos,pos2], [next_status,next_status_2], next_bitmap)
                                            else:        state =  self._int2extState([pos2,pos], [next_status_2,next_status], next_bitmap)
    
                                            if next_bitmap + np.sum([next_status, next_status_2]) == 0: reward = 100
                                            elif action == 6:                                           reward = -2
                                            else:                                                       reward = -3
    
                                            self.MDP[0][idx][a1*7+a2] = states_idx[state]
                                            self.MDP[1][idx][a1*7+a2] = reward

if __name__ == "__main__":

    # Robot
    #robot = Robot_2A2L()
    robot = Robot_YuMi()

    # Pieces configuration
    # ... for YuMi
    pieces = [{'start' : [-350, 450, 0],  # Piece 1
               'end'   : [ 350, 400, 0],
              },                     
              {'start' : [-350, 300, 0],  # Piece 2
               'end'   : [ 150, 300, 0],
              },                     
              {'start' : [-150, 450, 0],  # Piece 3
               'end'   : [ 250, 250, 0],
              },                     
              {'start' : [-50,  350, 0],  # Piece 4
               'end'   : [ 250, 500, 0],
              }]
    # ... for 2A2L
    #pieces = [{'start' : [-0.5, -0.5, 0],  # Piece 1
    #           'end'   : [ 0.5, -1.5, 0],
    #          },
    #          {'start' : [-0.3, -0.8, 0],  # Piece 2
    #           'end'   : [ 1,   -1,   0],
    #          },
    #          {'start' : [-0.3, -1,   0],  # Piece 3
    #           'end'   : [ 0.3, -1,   0],
    #          },
    #          {'start' : [-0.6, -1.3, 0],  # Piece 4
    #           'end'   : [ 0.6, -0.8, 0],
    #          }]


    mdp_test = mdp_generator(robot, pieces)

    if not mdp_test.load('MDP.bin'):
        mdp_test.generate()
        mdp_test.save('MDP.bin')

    #mdp_test.update(pieces)
    #mdp_test.update()

