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

    def __init__(self, robot, pieces, globalCfg):
        super().__init__(robot, pieces, globalCfg)

        filename = "MDP_RAW.bin" 
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

        print("Filtering valid states ...")

        # Filter out invalid states
        valid_states = [s for s in range(self.nS) if self._isStateValid(s)]
        valid_nS = len(valid_states)
        print("{} out of {}".format(valid_nS, self.nS))

        states_idx = {}
        for i, item in enumerate(valid_states):
            states_idx[item] = i

        print("Computing MDP ...")
        # Note: MDP is very large. Use numpy
        mdp_s = np.zeros((valid_nS, self.nA), dtype=np.int32) 
        mdp_r = np.zeros((valid_nS, self.nA), dtype=np.int16)
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
    
        offset_a = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,-1],[0,0,1]]) # left, rigth, back, fron, down, up
    
        p_ini = [pos for pos in self.piecesLocation['start']]
        p_end = [pos for pos in self.piecesLocation['end']  ]

        print(p_ini, p_end)
        states_idx = self.MDP[3]

        counter = 0

        #TODO Remove this debug stuff
        debug = False # Bypass pick_pos states
        
        # This loop iterates over all states where at least one arm (called 
        # fix-arm) is in position to pick/drop a piece.
        # This is a subset of all states, so faster then iterating over all states.
        # Note: suffix "_1" and "_2" refers to fix-arm and other-arm respectively.
        # 
        # Fix-arm can be the first or the second arm
        # ONLY_LEFT_ARM: run only arm==0

        # For one arm (arm1 or arm2) being on any of the init or end pieces position, iterate over
        # the rest of state combinations. Recall the MDP state formula:
        #
        # Total_states = (M*NZ + K*P)^2 * (K+1)^2 * (2+T)^K
        #              ____/       |          |         |
        #             /            |          |         |
        #       armsGridPos     pickPos   armsStatus   piecesStatus
        #
        #     Grid size: M*N*Z   --------------\ Hence the (M*N*Z + K*P)^2
        #     Extra pick/frop steps: K*P ----/
        #     Arm status: 0-K (0: empty arm, 1-K: arm carrying piece Ki) -> Hence the (K+1)^2
        #     Piece status: 0 (piece processed) or (piece not processed) -> Hence the 2^K
        #
        # For this arm, iterates over all pick/drop extra steps, and pick/drop
        # decision is based on the pieces status.
        #
        # For the other arm (arm2 or arm1), and pieces state vars, try all combinations
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
        # Fix-arm being left or right
        for arm in range(2):
            # Fix-arm state vars loop (only at pieces ini/end positions)
            for i, (pos_ini,pos_end) in enumerate(zip(p_ini, p_end)):
                for tmp_pp_1 in range(self.P+1):
                    # Other-arm state vars loop (at any position)
                    for xyzp_2 in range(self.M*self.N*self.Z + self.K*self.P):
                        for status_2 in range(self.K+1):
                            # Pieces state vars loop
                            for ps in range((2+self.T)**self.K):
    
                                # Piece status (from scalar to list)
                                pieces_status = []
                                for _ in range(self.K):
                                    tmp = ps % (2 + self.T)
                                    ps = (ps - tmp) // (2 + self.T)
                                    pieces_status.insert(0, tmp)
    
                                # Fix-arm
                                reduced_pp_1 = tmp_pp_1
                                if reduced_pp_1 > 0: pick_pos_1 = reduced_pp_1 + (i * self.P) # Range 0..K*P
                                else:                pick_pos_1 = 0
                                if pieces_status[i] == 1:
                                    pos = pos_ini
                                    status = 0
                                elif pieces_status[i] == 0:
                                    pos = pos_end
                                    status = i+1
                                else:
                                    # Not updating intermediate positions for Fix-arm
                                    continue
    
                                # Other-arm
                                if xyzp_2 < (self.M * self.N * self.Z):
                                    pos_2 = self._idx2xy(xyzp_2)
                                    pick_pos_2 = 0
                                else:
                                    pos_2 = [-1, -1, -1] # undefined
                                    pick_pos_2 = xyzp_2 + 1 - (self.M * self.N * self.Z)
    
                                # Get internal scalar state representation (from internal state vars)
                                if arm == 0: state = self._int2extState([pos, pos_2], [status, status_2], pieces_status, [pick_pos_1, pick_pos_2])
                                else:        state = self._int2extState([pos_2, pos], [status_2, status], pieces_status, [pick_pos_2, pick_pos_1])
     
                                # Check valid state
                                if state not in states_idx:
                                    continue
    
                                # Get MDP scalar state from internal scalar state
                                idx = states_idx[state]
    
                                # Taking action in Fix-arm
                                next_pieces_status = pieces_status[:]
    
                                USE_IMPLEMENTED_STEP_FUNCTION = 1
    
                                if USE_IMPLEMENTED_STEP_FUNCTION == 0:
                                    if status == 0:
                                        # Pick
                                        a = self.ACTION_PICK
                                        #TODO este trozo probablemente habra que borrarlo si pick_pos_1 es 0 siempre
                                        if reduced_pp_1 >= self.P or debug==True:
                                            next_pick_pos_1 = 0
                                            next_status = i+1
                                            next_pieces_status[i] = 0
                                        else:
                                            next_pick_pos_1 = reduced_pp_1 + 1 + self.P*i # Range 0..K*P
                                            next_status = status
                                    else:
                                        # Drop
                                        a = self.ACTION_DROP
                                        #TODO este trozo probablemente habra que borrarlo si pick_pos_1 es 0 siempre
                                        if reduced_pp_1 >= self.P or debug==True:
                                            next_status = 0
                                            next_pick_pos_1 = 0
                                        else:
                                            next_status = status
                                            next_pick_pos_1 = reduced_pp_1 + 1 + self.P*i # Range 0..K*P
                                    
                                # Taking action
                                a = self.ACTION_PICK if status == 0 else self.ACTION_DROP # Fix-arm action
    
                                for a_2 in range(self.single_nA): # Other_arm action
    
                                    if arm == 0: joint_a = [a, a_2]
                                    else:        joint_a = [a_2, a]
    
                                    action = self._int2extAction(joint_a)
    
                                    if self.MDP[1][idx][action] != -30:
                                        # Not marked to be updated
                                        continue
    
                                    if USE_IMPLEMENTED_STEP_FUNCTION:
                                        # Using _step() implementation (slower then code here after continue)
                                        self.reset(state)
                                        next_state, reward, _, _ = self._step(action)
                                        #print(state, action)
                                        #if self._isStateValid(next_state):
                                        if next_state in states_idx:
                                            self.MDP[0][idx][action] = states_idx[next_state]
                                            self.MDP[1][idx][action] = reward
                                            counter += 1
                                        # Done
                                        continue
    
                                    # remove mark
                                    self.MDP[1][idx][action] = -19
    
                                    next_status_2   = status_2
                                    next_pick_pos_2 = pick_pos_2
    
                                    # Move from 0..K*P to 0..P range
                                    if pick_pos_2 > 0: reduced_pp_2 = ((pick_pos_2 - 1) % self.P) + 1 # Range 0..P
                                    else:              reduced_pp_2 = 0
    
                                    if a_2 < 4: 
                                        if reduced_pp_2 > 0:
                                            # only pick/drop actions are valid
                                            continue
    
                                        pos2 = list(pos_2 + offset_a[a_2])
                                        if not self._isArmsGridPosValid2(pos2):
                                        # TODO cambiar esto por funcion existetne que ya lo chequea
                                        #if pos2[0] >= self.M or pos2[0] < 0 or pos2[1] < 0 or pos2[1] >= self.N or pos2[2] >= self.Z or pos2[2] < 0:
                                            continue
                                    else:
                                        pos2 = pos_2
                                        if a_2 == 4:
                                            if (status_2 == 0):
                                                # Get the piece index
                                                if reduced_pp_2 > 0:
                                                    # Derive piece index from pick_pos value (pos_2 has no meaning)
                                                    p_idx = (next_pick_pos_2 - 1) // self.P
                                                else:
                                                    # Derive piece index from location
                                                    if (pos_2 in p_ini):
                                                        p_idx = p_ini.index(pos_2)
                                                    else:
                                                        continue
                                                
                                                #TDOOO no tengo nada claro que hacer si esto vale 2 pero no es un pick or un drop
                                                if pieces_status[p_idx] > 1:
                                                    # extra intermediate positions already done
                                                    continue
                                                elif pieces_status[p_idx] > 0:
                                                    if reduced_pp_2 >= self.P or debug==True: # Range 0..P
                                                        next_pick_pos_2 == 0
                                                        next_status_2 = p_idx+1
                                                        next_pieces_status[p_idx] = 0
                                                    else:
                                                        next_pick_pos_2 += 1 # Range 0..K*P
                                                else:
                                                    continue
                                            else:
                                                continue
    
                                        elif a_2 == 5:
                                            if (status_2 > 0):
                                                # Get the piece index
                                                if reduced_pp_2 > 0:
                                                    # Derive piece index from pick_pos value (pos_2 has no meaning)
                                                    p_idx = (next_pick_pos_2-1)//self.P
                                                else:
                                                    # Derive piece index from location
                                                    if (pos_2 in p_end):
                                                        p_idx = p_end.index(pos_2)
                                                    else:
                                                        continue
                                                
                                                if pieces_status[p_idx] > 1:
                                                    continue
                                                elif (pieces_status[p_idx] > 0) and (status_2 == (p_idx+1)):
                                                    if reduced_pp_2 >= self.P or debug==True:
                                                        next_pick_pos_2 == 0
                                                        next_status_2 = 0
                                                    else:
                                                        next_pick_pos_2 += 1
                                                else:
                                                    continue
                                            else:
                                                continue
    
                                        else:
                                            if reduced_pp_2 > 0:
                                                continue
                                    #print("AA ", arm, joint_a)
                                    #print(pos)
                                    #print(self._ext2intState(state))
                                    if arm == 0:
                                        next_arms_grid_pos, next_arms_status, next_pieces_status, next_pick_pos = [pos,pos2], [next_status,next_status_2], next_pieces_status, [next_pick_pos_1, next_pick_pos_2]
                                    else:
                                        next_arms_grid_pos, next_arms_status, next_pieces_status, next_pick_pos = [pos2,pos], [next_status_2,next_status], next_pieces_status, [next_pick_pos_2, next_pick_pos_1]
                                    if self.robot.checkValidLocation(next_arms_grid_pos) == False:
                                        continue
                                    if self.robot.checkCollision(next_arms_grid_pos) == True:
                                        continue
                                    if self._isPiecesStatusValid(next_arms_status, next_pieces_status, next_pick_pos) == False:
                                        continue
    
                                    # update next_state and reward
                                    if arm == 0: state = self._int2extState([pos,pos2], [next_status,next_status_2], next_pieces_status, [next_pick_pos_1, next_pick_pos_2])
                                    else:        state = self._int2extState([pos2,pos], [next_status_2,next_status], next_pieces_status, [next_pick_pos_2, next_pick_pos_1])
                                    #print(pick_pos, next_pick_pos)
    
                                    # Check if the new state is valid
                                    if state not in states_idx:
                                        continue
    
                                    if (np.sum(next_pieces_status) + np.sum([next_status, next_status_2])) == 0: reward = 10000 #TODO algo raro pasa...con 200 no va y con 100 va mas o menos
                                    elif a_2 == self.ACTION_STAY:                                                reward = -1 # 1 arm
                                    else:                                                                        reward = -1 # 2 arms
    
                                    #print(self._ext2intState(state))
                                    counter += 1  
                                    self.MDP[0][idx][action] = states_idx[state]
                                    self.MDP[1][idx][action] = reward
        print("COUNTER {}\n".format(counter))

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

