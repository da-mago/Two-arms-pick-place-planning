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

    def __init__(self, robot, pieces):
        super().__init__(robot, pieces)

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
               #next_state, reward, done, info = self._step(a, mode=1)
               next_state, reward, done, info = self._step(a, mode=0)
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
    
        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
    
        p_ini = [pos for pos in self.piecesLocation['start']]
        p_end = [pos for pos in self.piecesLocation['end']  ]

        print(p_ini, p_end)
        states_idx = self.MDP[3]


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
        # Total_states = ((M*N + K*P)^2) * ((K+1)^2) * 2^K
        #
        #     Grid size: M*N   --------------\ Hence the (M*N + K*P)^2
        #     Extra pick/frop steps: K*P ----/
        #     Arm status: 0-K (0: empty arm, 1-K: arm carrying piece Ki) -> Hence the (K+1)^2
        #     Piece status: 0 (piece processed) or (piece not processed) -> Hence the 2^K
        #
        # For this arm, iterates over all pick/drop extra steps, and pick/drop
        # decision is based on the pieces status (bitmap).
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
        for arm in range(2):
            # Fix-arm configuration: pos, pick_pos and  (status is derived from pieces bitmap)
            for i, (pos_ini,pos_end) in enumerate(zip(p_ini, p_end)):
                for tmp_pick_pos_1 in range(self.P+1):
                    # Other-arm configuration: pos, pick_pos  and status
                    for xyp in range(self.M*self.N + self.K*self.P):
                        for status_2 in range(self.K+1):
                            # Arm independent configuration: pieces bitmap
                            for bitmap in range(2**(self.K)):

                                # Other-arm
                                if xyp < self.M*self.N:
                                    x = xyp%self.M
                                    y = xyp//self.M
                                    pick_pos_2 = 0
                                else:
                                    x = -1
                                    y = -1
                                    pick_pos_2 = xyp+1 - self.M*self.N
                                
                                # Fix-arm
                                # (Using a temporal var because if I modify the loop var, memory is overwritten)
                                pick_pos_1 = tmp_pick_pos_1
                                if bitmap & (1<<i):
                                    # pick
                                    pos = pos_ini
                                    status = 0
                                    if pick_pos_1 >= self.P or debug==True:
                                        next_pick_pos_1 = 0
                                        next_status = i+1
                                        tmp_bitmap = bitmap & ~(1<<i)
                                    else:
                                        next_pick_pos_1 = pick_pos_1 + 1 + self.P*i # Range 0..K*P
                                        next_status = status
                                        tmp_bitmap = bitmap
                                    a = 4
                                    # place
                                else:
                                    # drop
                                    pos = pos_end
                                    status = i+1
                                    tmp_bitmap = bitmap
                                    if pick_pos_1 >= self.P or debug==True:
                                        next_status = 0
                                        next_pick_pos_1 = 0
                                    else:
                                        next_status = status
                                        next_pick_pos_1 = pick_pos_1 + 1 + self.P*i # Range 0..K*P
                                    a = 5
                                    
                                # Change range from 0..P to 0..K*P
                                if pick_pos_1 > 0:
                                    pick_pos_1 += self.P*i # Range 0..K*P

                                if arm == 0:
                                    state = self._int2extState([pos, [x,y]], [status, status_2], bitmap, [pick_pos_1, pick_pos_2])
                                else:
                                    state = self._int2extState([[x,y], pos], [status_2, status], bitmap, [pick_pos_2, pick_pos_1])
    
                                #if pos == [0,3] and [x,y] == [8,2] and status==0 and status_2==0 and bitmap==15 and pick_pos_1 ==0 and pick_pos_2 == 0:
                                #if pos == [0,3] and [x,y] == [8,2] and status==0 and status_2==0 and bitmap==15 and pick_pos_1 ==2 and pick_pos_2 == 0:
                                #if pos == [1,4] and status_2==1 and bitmap==8 and pick_pos_2 == 1:
                                #print(pos, self._ext2intState(state))
                                # Process only valid states
                                if state in states_idx:
                                    
                                    tmp_pick_pos_2 = pick_pos_2
                                    idx = states_idx[state]
                                    # Avoid checks (it was done in phase 1, -30 means all checks were passed)
                                    for action in range(7):
                                        #print(action)
                                        if arm == 0: a1 = a;      a2 = action;
                                        else:        a1 = action; a2 = a;


                                        # Emulate robot with single arm (the other arm ca not pick/drop)
                                        #if a2==4 or a2==5:
                                        #   continue

                                        next_bitmap = tmp_bitmap
                                        if self.MDP[1][idx][a1*7+a2] == -30: # pick/any
                                            # Using _step() implementation (slower then code here after continue)
                                            #self.reset(state)
                                            #next_state, reward, _, _ = self._step(a1*7+a2)
                                            #self.MDP[0][idx][a1*7+a2] = states_idx[next_state]
                                            #self.MDP[1][idx][a1*7+a2] = reward
                                            #continue

                                            # remove mark
                                            self.MDP[1][idx][a1*7+a2] = -19
    
                                            next_status_2   = status_2
                                            pick_pos_2      = tmp_pick_pos_2 # Range 0..K*P
                                            next_pick_pos_2 = pick_pos_2

                                            # Move from 0..K*P to 0..P range
                                            if pick_pos_2 > 0:
                                                pick_pos_2 = ((pick_pos_2-1) % self.P) + 1 # Range 0..P

                                            if action < 4: 
                                                if pick_pos_2 > 0:
                                                    # only pick/drop actions are valid
                                                    continue

                                                pos2 = list([x,y] + offset_a[action])
                                                if pos2[0] >= self.M or pos2[0] < 0 or pos2[1] < 0 or pos2[1] >= self.N:
                                                    continue
                                            else:
                                                pos2 = [x,y]
                                                if action == 4:
                                                    if (status_2 == 0):
                                                        # Get the piece index
                                                        if pick_pos_2 > 0:
                                                            # Derive piece index from pick_pos value (x,y has no meaning)
                                                            p_idx = (next_pick_pos_2-1)//self.P
                                                        else:
                                                            # Derive piece index from location
                                                            if ([x,y] in p_ini):
                                                                p_idx = p_ini.index([x,y])
                                                            else:
                                                                continue
                                                        
                                                        if bitmap & (1 << p_idx):
                                                            if pick_pos_2 >= self.P or debug==True: # Range 0..P
                                                                next_pick_pos_2 == 0
                                                                next_status_2 = p_idx+1
                                                                next_bitmap &= ~(1<<p_idx)
                                                            else:
                                                                next_pick_pos_2 += 1 # Range 0..K*P
                                                        else:
                                                            continue
                                                    else:
                                                        continue

                                                elif action == 5:
                                                    if (status_2 > 0):
                                                        # Get the piece index
                                                        if pick_pos_2 > 0:
                                                            # Derive piece index from pick_pos value (x,y has no meaning)
                                                            p_idx = (next_pick_pos_2-1)//self.P
                                                        else:
                                                            # Derive piece index from location
                                                            if ([x,y] in p_end):
                                                                p_idx = p_end.index([x,y])
                                                            else:
                                                                continue
                                                        
                                                        if (bitmap & (1 << p_idx)) == 0 and status_2 == (p_idx+1):
                                                            if pick_pos_2 >= self.P or debug==True:
                                                                next_pick_pos_2 == 0
                                                                next_status_2 = 0
                                                            else:
                                                                next_pick_pos_2 += 1
                                                        else:
                                                            continue
                                                    else:
                                                        continue
    
                                                else:
                                                    if pick_pos_2 > 0:
                                                        continue
                                            #print("AA ", arm, a1, a2)
                                            #print(pos)
                                            #print(self._ext2intState(state))
                                            if arm == 0:
                                                next_arms_grid_pos, next_arms_status, next_pieces_map, next_pick_pos = [pos,pos2], [next_status,next_status_2], next_bitmap, [next_pick_pos_1, next_pick_pos_2]
                                            else:
                                                next_arms_grid_pos, next_arms_status, next_pieces_map, next_pick_pos = [pos2,pos], [next_status_2,next_status], next_bitmap, [next_pick_pos_2, next_pick_pos_1]
                                            if self.robot.checkValidLocation(next_arms_grid_pos) == False:
                                                continue
                                            if self.robot.checkCollision(next_arms_grid_pos) == True:
                                                continue
                                            if self._isPiecesStatusValid(next_arms_status, next_pieces_map, next_pick_pos) == False:
                                                continue

                                            # update next_state and reward
                                            if arm == 0: state = self._int2extState([pos,pos2], [next_status,next_status_2], next_bitmap, [next_pick_pos_1, next_pick_pos_2])
                                            else:        state = self._int2extState([pos2,pos], [next_status_2,next_status], next_bitmap, [next_pick_pos_2, next_pick_pos_1])
                                            #print(pick_pos, next_pick_pos)

                                            # Check if the new state is valid
                                            if state not in states_idx:
                                                continue

                                            if next_bitmap + np.sum([next_status, next_status_2]) == 0: reward = 10000 #TODO algo raro pasa...con 200 no va y con 100 va mas o menos
                                            elif action == 6:                                           reward = -1 # 1 arm
                                            else:                                                       reward = -1 # 2 arms

                                            #print(self._ext2intState(state))
                                            self.MDP[0][idx][a1*7+a2] = states_idx[state]
                                            self.MDP[1][idx][a1*7+a2] = reward

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

