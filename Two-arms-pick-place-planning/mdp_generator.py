###############################################################################
#
# Convert any deterministic RL environment to a tabular MDP.
# Note: that speeds up the computation when searching for an optimal solution.
#
# Moreover, a deterministic MDP can be reduced to a graph. That allows you to 
# apply both graph search and RL algorithms to it.
#

from env_pickplace import env_pickplace
import numpy as np
import pickle

class mdp_generator(env_pickplace):

    def __init__(self, robot, pieces, globalCfg, filename):
        super().__init__(robot, pieces, globalCfg)

        # Load MDP template from file
        if not self._load(filename):
            # Create MDP template and save it to file
            self._generate()
            self._save(filename)

    def _load(self, filename):
        ''' Load MDP from file '''
        try:
            with open(filename, 'rb') as f:
                self.MDP = pickle.load(f)
            return True
        except:
            return False 

    def _save(self, filename):
        ''' Save MDP to file '''
        with open(filename, 'wb') as f:
            pickle.dump(self.MDP, f)

    def _generate(self):
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

        # Filter out invalid states
        print(" Brute states {}".format(self.nS))
        valid_states = [s for s in range(self.nS) if self._isStateValid(s, mode=env_pickplace.MODE_OFFLINE)]
        valid_nS = len(valid_states)
        print(" net states {}".format(valid_nS))
        print("Num states : {} (out of {})".format(valid_nS, self.nS))
        print("Num actions: {}".format(self.nA))

        states_idx = {}
        for i, item in enumerate(valid_states):
            states_idx[item] = i

        print("Computing MDP ", end='', flush=True)
        # Note: MDP is very large. Use numpy
        mdp_s = np.zeros((valid_nS, self.nA), dtype=np.uint32) 
        mdp_r = np.zeros((valid_nS, self.nA), dtype=np.int16)
        mdp_v = np.array(valid_states)
        mdp_i = {x:i for i,x in enumerate(mdp_v)}
        for i,s in enumerate(valid_states):
            for a in range(self.nA):
                #                if s != 288964: continue
                self.reset(s)
                next_state, reward, done, info = self._step(a, mode=1)
                mdp_s[i][a] = states_idx[next_state]
                mdp_r[i][a] = reward
            if (i % (valid_nS//100)) == 0:
                print("." , end='', flush=True)
                #print(10*i//(valid_nS//10),'%')

        # State space layers partition
        # TODO: prioritized_piecesMap must be generic fr n-pieces
        # No usado ahora mismo
        mdp_l = []
        #prioritized_piecesMap = [0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0x7, 0xb, 0xd, 0xe, 0xf]
        #prioritized_status    = [[0,0],[1,0],[2,0],[3,0],[4,0],[0,1],[0,2],[0,3],[0,4],[1,2],[1,3],[1,4],[2,1],[2,3],[2,4],[3,1],[3,2],[3,4],[4,1],[4,2],[4,3]]
        #piecesMap_len = len(prioritized_piecesMap)
        #status_len    = len(prioritized_status)
        #mdp_l = [[] for _ in range(piecesMap_len * status_len)]
        #for i, state in enumerate(mdp_v):
        #    armsGridPos, armsStatus, piecesMap, pickPos = self._ext2intState(state)
        #    #idx = prioritized_status.index(armsStatus) + status_len*prioritized_piecesMap[piecesMap]
        #    idx = prioritized_status.index(armsStatus) + status_len*prioritized_piecesMap.index(piecesMap)
        #    mdp_l[idx].append(i)

        self.MDP = [mdp_s, mdp_r, mdp_v, mdp_i, mdp_l] # next_state, reward, state reduction mapping, inverse state reduction mapping, layered partitions

    def update(self):
        ''' Update MDP for a specific configuration of pieces '''

        print("UPDATE MDP")
        offset_a = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,-1],[0,0,1]]) # left, rigth, back, fron, down, up

        p_ini = [pos for pos in self.piecesLocation['start']]
        p_end = [pos for pos in self.piecesLocation['end']  ]
        #p_ini = [self._updatePosFromPiece(pos) for pos in self.piecesLocation['start']]
        #p_end = [self._updatePosFromPiece(pos) for pos in self.piecesLocation['end']  ]
        #print(p_ini, p_end)

        states_idx = self.MDP[3]

        counter = 0

        # This loop iterates over all states where at least one arm (called 
        # fix-arm) is in position to pick/drop a piece.
        # This is a subset of the whole state-space, so faster then iterating over all states.
        # Note: suffix "_1" and "_2" refers to fix-arm and other-arm respectively.

        # Fix arm (may be either the left or the right one) can be on any of the
        # init or end pieces position. The other arm may be in any situation.
        # Recall the MDP state formula:
        #
        # Total_states = (M*N*Z + K*P)^2 * (K+1)^2 * (2+T)^K
        #              ____/       |         |         |
        #             /            |         |         |
        #       armsGridPos     pickPos   armsStatus   piecesStatus
        #
        #     Grid size: M*N*Z   ------------\ Hence the (M*N*Z + K*P)^2
        #     Extra pick/frop steps: K*P ----/
        #     Arm status: 0-K (0: empty arm, 1-K: arm carrying piece Ki) -> Hence the (K+1)^2
        #     Piece status: 0 (piece processed), 1 (piece not processed) or >=2 to indicate intermediate (T-2) state -> Hence the (2+T)^K
        #
        # Fix arm iterates over all pick/drop extra steps, and pick/drop
        # feasibility decision is based on the pieces status.
        #
        # The other arm (right or left), and other pieces state vars, take all combininated values
        #
        # Then for each of the previous valid states (not all of them, only the
        # valida ones), iterate ovar all possible actions in the 'other arm'.
        # The resulting state for the taken action is also validated,
        # 
        # There is an extra check for the initial state: was marked when the
        # model was generatedr? Otherwise, it will ignore it.
        #
        # For all that remaining valid scenarios, it will compute the
        # appropriate next state and reward, since at this point the pieces init
        # and end location is known.
        #
        cnt = 0
        cnt2 = 0
        # Fix-arm being left or right
        for arm in range(2):
            # Fix-arm state vars loop (only at pieces ini/end positions)
            for i, (pos_ini,pos_end) in enumerate(zip(p_ini, p_end)):
                for tmp_pp_1 in range(self.P+1):
                    # Other-arm state vars loop (at any position)
                    for xyzp_2 in range(self.M*self.N*self.Z + self.K*self.P):
                        for status_2 in range(self.K+1):
                            # Pieces state vars loop
                            for ps_tmp in range((2+self.T)**self.K):

                                for time_step in range(self.PS):
                                    ps = ps_tmp
                                   # print(ps, time_step)
                                    # Piece status (from scalar to list)
                                    pieces_status = []
                                    for _ in range(self.K):
                                        tmp = ps % (2 + self.T)
                                        ps = (ps - tmp) // (2 + self.T)
                                        pieces_status.insert(0, tmp)

                                    # Fix-arm
                                    reduced_pp_1 = tmp_pp_1
                                    if reduced_pp_1 > 0: pick_pos_1 = reduced_pp_1 + (i * self.P) # Range 0..K*o
                                    else:                pick_pos_1 = 0
                                    if pieces_status[i] == 1:
                                        pos, _ = self._piece2robotPos(pos_ini, time_step, reduced_pp_1)
                                        #DMG si el segundo argumento devuelto es False, we can skip it (continue)
                                        status = 0
                                    elif pieces_status[i] == 0:
                                        pos, _ = self._piece2robotPos(pos_end, 0, 0)
                                        status = i+1
                                    else:
                                        # Not updating intermediate positions for Fix-arm
                                        continue

                                    # Other-arm
                                    if xyzp_2 < (self.M * self.N * self.Z):
                                        pos_2 = self._idx2xy(xyzp_2)
                                        pick_pos_2 = 0
                                    else:
                                        #pos_2 = [-1, -1, -1] # undefined
                                        pos_2 = self.robot.unknown_pos
                                        pick_pos_2 = xyzp_2 + 1 - (self.M * self.N * self.Z)

                                    ## Get internal scalar state representation (from internal state vars)
				    ## DMG esto hay que pensarlo bien
                                    if time_step < (self.PS - 1):
                                        next_time_step = time_step + 1
                                    else:
                                        next_time_step = time_step
#                                        continue
#                                    next_time_step = 0

                                    if arm == 0: state = self._int2extState([pos, pos_2], [status, status_2], pieces_status, [pick_pos_1, pick_pos_2], time_step)
                                    else:        state = self._int2extState([pos_2, pos], [status_2, status], pieces_status, [pick_pos_2, pick_pos_1], time_step)

                                    # Check valid state
                                    if state not in states_idx:
                                        continue

                                    # Get MDP scalar state from internal scalar state
                                    idx = states_idx[state]

                                    # Taking action in Fix-arm
                                    next_pieces_status = pieces_status[:]

                                    # Taking action
                                    a = self.ACTION_PICK if status == 0 else self.ACTION_DROP # Fix-arm action

                                    for a_2 in range(self.single_nA): # Other_arm action

                                        if arm == 0: joint_a = [a, a_2]
                                        else:        joint_a = [a_2, a]

                                        action = self._int2extAction(joint_a)

## debug info
                                        if self.MDP[1][idx][action] != -30:
                                            #raise Exception("HOLA")
                                            cnt2 += 1
                                            continue
                                            

                                        cnt += 1
                                        self.reset(state)
                                        next_state, reward, _, _ = self._step(action)
                                        if next_state in states_idx:
                                            self.MDP[0][idx][action] = states_idx[next_state]
                                            self.MDP[1][idx][action] = reward
                                            counter += 1

        print("CNT      :", cnt)
        print("CNT2     :", cnt2)
        print("CNT (-30):", np.count_nonzero(self.MDP[1] == -30))



if __name__ == "__main__":

    # YuMi robot
    if True:
        from robot_YuMi import Robot_YuMi

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

    # Simulated 2A2L
    #else:
    #    from robot_2A2L import Robot_2A2L
    #    
    #    robot = Robot_2A2L()
    #    
    #    pieces = [{'start' : [-0.5, -0.5, 0],  # Piece 1
    #               'end'   : [ 0.5, -1.5, 0],
    #              },
    #              {'start' : [-0.3, -0.8, 0],  # Piece 2
    #               'end'   : [ 1,   -1,   0],
    #              },
    #              {'start' : [-0.3, -1,   0],  # Piece 3
    #               'end'   : [ 0.3, -1,   0],
    #              },
    #              {'start' : [-0.6, -1.3, 0],  # Piece 4
    #               'end'   : [ 0.6, -0.8, 0],
    #              }]


    mdp_test = mdp_generator(robot, pieces)

    if not mdp_test.load('MDP.bin'):
        mdp_test.generate()
        mdp_test.save('MDP.bin')

    #mdp_test.update(pieces)
    mdp_test.update()

