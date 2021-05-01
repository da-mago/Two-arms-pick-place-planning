#
# The idea of decomposing the problem hierarchically is to solve a simpler problem
#
# Introduction
# ------------
# The problem definition: a robot with 2 arms moving 4 pieces from their initial to final position
#
# The original (RAW) approach is to define the MDP state space this way:
# S = (XY arm position ^ number of arms) x Pieces_bitmap (4bits) x (Piece_status (0-5) ^ number of arms)
# S = 50^2 x 16 x 5^2 = 1 million
# 
# Note: actually, many of those states make no sense. In the end, the RAW MDP, after filtering out those invalid
#       states have about 140 mil states.
#
# On the other hand, the action space was the joint action space of both arms
# A = number of actions ^ 2 = 49
#
# Note actually, we only consider 48 out of 49 actions, since not moving for both arms make no sense.
#
# Hierarchical decomposition
# --------------------------
# Call it decomposing a hard problem in to simpler ones, or divide and conquer, or whatever you want. The idea is to
# end up with a smaller MDP to solve.
# We create two layers:
# - High layer: the one to solve in real time
#               Encodes abstracted actions, like 'arm1 go to piece 3' and abstracted states, like 'arm1 in piece3 locatiom'
# - Low layer : the one to precompute offline
#               Encodes how to go from Ai to Bi for both arms. There a total of 2500 independent low layer MPDs (one for each 
#               combination of arms XY destination)
#
# Low layer
# ---------
# State space  -> S = Grid size ^ 2 = 2500
# Action space -> A = Actions (up/down/left/right/stay) ^ 2 = 25
# Note: one MDP per arms destination combination (so 2500 MDPs to solve offline). Both policy and accumulated reward shall be stored.
#
# High layer
# ----------
# State space -> S = Pieces_bitmap x (arm1_state ^ num_arms) = 16*(5^2) = 400
#   
#   where
#       Pieces bitmap: 4 bits, one per piece (it means a piece taken or not)
#       armx_state   : 0 (no carrying any piece), 1-4 (carrying a piece)
#
# Action space -> A = Arm_actions ^ 2 = 25
#
#   where
#       Arm_actions  : Go to piece 1,2,3 or 4, Go to piece destination
#
# Note: a single MDP to solve online
#
# Approach issues:
# - Let's perform this joint action (arm1 go to piece 1, and arm2 go to piece 2). There could happen one of these 3 things: arm1 reach its target first, or maybe arm2 or maybe both at the same time.
#   Eacn of the three cases represent a diferent state in the MDP. Depending on the initial position of the arms, one of the three cases will happen. This is why the MDP definition must be done in 
#   real time (once you know the initial arms position and pieces position). Offline low layer MDPs will be used to check which of the three cases will happen.
# 
#                                /---> Low chance that both arms arrive at the same time. This state will either represent
#                               /      arm1 and arm2 in their target positions (S3), or only arm1 (S1) or arm2 (S2) in its 
#                               |      target position and the other one in its way to the piece.
#                               |      Note: The transition to S2, S3 or S4 will be known in real time, based on the initial 
#        arm1 go to piece 1     |            pieces pos and the offline computed solutions from lower layers
#        arm2 go to piece 2     |             
# S0 -------------------------> S1 ---arm1 pick piece 1, arm 2 go to piece 2----------------------> S4 ---arm1 go to piece 3, arm 2 go to piece 2-------------> S7 ---arm1 pick piece 3, arm2 go to piece 2 ----.....
# | \                           S2 ---arm2 pick piece 2, arm 1 go to piece 1--------->              S5 ---arm1 pick piece 1, arm 2 pick piece 2------>          S8 ---arm1 go to piece 3, arm2 pick piece 2
# \  \   arm1 go to piece 1     S3 ---arm1 pick piece 1, arm 2 pick piece 2---------->              S6 ---arm1 go to piece 3, arm 2 go to piece 4---->          S9 ---arm1 pick piece 3, arm2 pick piece 2
#  \  \  arm2 go to piece 2 
#   \  \--------------------->
#    \
#     \  Other joint actions
#      \---- ......
#
# S0: 1111 (all pieces at origin), 0 (arm1 not carrying a piece), 0 (arm1 not carrying a piece)
# S1: 1111, 1, 0 (arm1 at piece 1 location)
# S2: 1111, 0, 2 (arm2 at piece 2 location)
# S3: 1111, 1, 2 (arm1 at piece 1 location and arm2 at piece 2 location)
# S4: 0111, 1, 0 (arm1 picked piece 1)
# S5: 1111, 0, 2 (arm2 at piece 2 location)
# S6: 0111, 1, 2 (arm1 picked piece 1 and arm2 at piece 2 location)
# S7: 0111, 1, 0 (arm1 picked piece 1)
# S8: 1111, 0, 2 (arm2 at piece 2 location)
# S9: 0111, 1, 2 (arm1 picked piece 1 and arm2 at piece 2 location)
#
# For example, in this tree-like scheme, let's say that our MDP (build in real time consists of S0->S1->S4->S7->... , but S2,S3,S5,S6 wont belong to it.
# Note: this is assuming that the system is deterministic, so precomputed policies will show us the MDP to build.
#
# - The pick/drop actions duration is N time steps. Example: S1->S4 (arm1 pick piece before arm2 reaching piece 2), but S1->S5 (arm2 reaches piece 2 before arm1 completes pick action)
#
# Let's develop a single path (there are other 11 possible starting actions) up to the end
#
#             (there are other 11 possible actions)                 (the unique possible joint action)                   (arm1 could also go to p4)                            (the unique possible joint action)
# 1111/0/0 ---arm1 go to piece 1, arm2 go to piece 2--> 1111/1/0 ---arm1 pick piece 1, arm2 go to piece 2--> 0111/0/0 ---arm1 go to piece 3, arm2 go to piece 2--> 0111/0/2 ---arm1 go to piece 3, arm2 pick piece 2--> 0011/0/2 --
#             (the unique possible joint action)                    (the unique possible joint action)                   (the unique possible joint action)                    (the unique possible joint action)
#          ---arm1 go to piece 3, arm2 go to piece 4--> 0011/3/0 ---arm1 pick piece 3, arm2 go to piece 4--> 0011/3/4 ---arm1 pick piece 3, arm2 pick piece 4----> 0001/0/4 ---arm1 stay, arm2 pick piece 4-----------> 0000/0/0
# 
# Deph -> 8 steps, so 12 possible actions from initial step it makes a total of 12*8=96 states MDP. Actually more than 96, since in the above example, you can see how at some point can choose between go to piece 3 or 4 (so the other
# branch adds extra new states). 


# Other option: each low MDP includes MOVE+PICK action, so let's say pick/drop actions take 8 time steps -> S = (50+8)^2 = 3364 states
# Same example:
#             (there are other 11 possible actions)         (arm1 could also go to p4)                        (the unique possible joint action)                (the unique possible joint action)
# 1111 ---arm1 go to piece 1, arm2 go to piece 2--> 0111 ---arm1 go to piece 3, arm2 go to piece 2--> 0011 ---arm1 go to piece 3, arm2 go to piece 4--> 0001 ---arm1 stay, arm2 pick piece 4--> 0000
#
# Note: it seems a much simpler solution

from env_hrl import env_hrl
from mdp_solver import mdp_solver
import numpy as np
import pickle
import time

class mdp_hrl_generator(env_hrl):
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

        print('Total states:', self.nS)

        # Filter out invalid states
        valid_states = [s for s in range(self.nS) if self._isStateValid(s)]
        valid_nS = len(valid_states)
        print('Valid states:', valid_nS)

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
               next_state, reward, done, info = self.step(a)
               mdp_s[i][a] = states_idx[next_state]
               mdp_r[i][a] = reward
            if (i % (valid_nS//10)) == 0:
                print(10*i//(valid_nS//10),'%')

        # State space layers partition
        # TODO: prioritized_piecesMap must be generic fr n-pieces
        #prioritized_piecesMap = [0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0x7, 0xb, 0xd, 0xe, 0xf]
        #prioritized_status    = [[0,0],[1,0],[2,0],[3,0],[4,0],[0,1],[0,2],[0,3],[0,4],[1,2],[1,3],[1,4],[2,1],[2,3],[2,4],[3,1],[3,2],[3,4],[4,1],[4,2],[4,3]]
        #piecesMap_len = len(prioritized_piecesMap)
        #status_len    = len(prioritized_status)
        #mdp_l = [[] for _ in range(piecesMap_len * status_len)]
        #for i, state in enumerate(mdp_v):
        #    armsGridPos, armsStatus = self._ext2intState(state)
        #    #idx = prioritized_status.index(armsStatus) + status_len*prioritized_piecesMap[piecesMap]
        #    idx = prioritized_status.index(armsStatus) + status_len*prioritized_piecesMap.index(piecesMap)
        #    mdp_l[idx].append(i)
        mdp_l = []

        self.MDP = [mdp_s, mdp_r, mdp_v, mdp_i, mdp_l] # next_state, reward, state reduction mapping, inverse state reduction mapping, layered partitions


    def update(self):
        ''' Update MDP for a specific configuration of pieces '''
    
        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
    
        states_idx = self.MDP[3]
    
        # For each state where:
        # - arm0 is picking up or dropping off its pieces, and
        # - arm1 is anywhere (x,y,c)
        # compute the resulting state and reward for any joint action (where arm0 is pick or drop, and arm1 is any action)
        #
        # Note: IDEM in the opposite case (arm1 is picking up or dropping down and arm0 is anywhere)
        # Note: Everything is implemented in a single common code (this is why there so many IF p....)
        #
        pos_pieces = [self.piecesLocation['start'][0], self.piecesLocation['end'][0], self.piecesLocation['start'][1], self.piecesLocation['end'][1]]

        for x in range(self.M):
            for y in range(self.N):
                for c_any in range(2):

                    pos_any = [x,y]

                    for p, pos in enumerate(pos_pieces):
                        if p<2:
                            armsPos = [pos, pos_any]
                            armsStatus = [p%2, c_any]
                        else:
                            armsPos = [pos_any, pos]
                            armsStatus = [c_any, p%2]
                        state = self._int2extState(armsPos, armsStatus)
                        #print(state)

                        # Filter out invalid states
                        if state in states_idx:
                            idx = states_idx[state]

                            # For each action over pos_any location
                            for action_any in range(7):
                                if p == 0:
                                    a1 = 4
                                    a2 = action_any
                                    c_fix = 1
                                elif p == 1:
                                    a1 = 5
                                    a2 = action_any
                                    c_fix = 0
                                elif p == 2:
                                    a2 = 4
                                    a1 = action_any
                                    c_fix = 1
                                elif p == 3:
                                    a2 = 5
                                    a1 = action_any
                                    c_fix = 0

                                c_any_next = c_any
                                # Move
                                if action_any < 4:
                                    pos_any_next = list(pos_any + offset_a[action_any])

                                    # Validate pos or use a predefined mark to accept this state-action
                                    if not self._isArmsGridPosValid(pos_any_next):
                                        continue

                                    # Validar reachabilidad??
                                    if not robot.reachable[0, self._xy2idx(pos_any_next)]:
                                        continue

                                else:
                                    pos_any_next = pos_any

                                    # Pick
                                    if action_any == 4:
                                        if c_any == 0 and pos_any == pos_pieces[(p<2)*2]:
                                            c_any_next = 1 # piece picked up
                                        else:
                                            continue
                                    # Drop
                                    elif action_any == 5:
                                        if c_any == 1 and pos_any == pos_pieces[(p<2)*2+1]:
                                            c_any_next = 0 # piece dropped off
                                        else:
                                            continue


                                # update next_state and reward
                                if p<2:
                                    armsPos_next = [pos, pos_any_next]
                                    armsStatus_next = [c_fix, c_any_next]
                                else:
                                    armsPos_next = [pos_any_next, pos]
                                    armsStatus_next = [c_any_next, c_fix]
                                state = self._int2extState(armsPos_next, armsStatus_next)

                                if not self._isStateValid(state):
                                    #print([pos, self.piecesLocation['end'][1]])
                                    #print(a1, a2)
                                    continue

                                # Compute reward
                                # TODO: review
                                #if armsPos_next == [pos_pieces[1], pos_pieces[3]]:
                                #if (np.sum(armsStatus_next) == 0) and (armsPos_next[0] == pos_pieces[1]) and (armsPos_next[1] == pos_pieces[3]): reward = 100
                                #if (np.sum(armsStatus_next) == 0) and (armsPos_next[0] == pos_pieces[0]) and (armsPos_next[1] == pos_pieces[2]):
                                if (a1 == 5 and (armsPos_next[0] == pos_pieces[1]) ) or (a2 == 5 and (armsPos_next[1] == pos_pieces[3]) ):
                                    reward = 100
                                #if self._isGoalMet():  reward = 100 # Goal reached
                                elif action_any == 6:  
                                    reward = -2  # only one arm moving
                                else:                  reward = -3  # both arms moving

                                # Update next_state/reward
                                self.MDP[0][idx][a1*7+a2] = states_idx[state]
                                self.MDP[1][idx][a1*7+a2] = reward


def getReturn(mdp, policy, state):
    ''' Given the MDP and the computed policy, get the Return (cumulative reward)
        until any of the arms complete its task (pick and place its piece) '''

    R = 0
    #while True:
    for i in range(20):

        action  = policy[state]
        joint_a = mdp._ext2intAction(action)
        #R      += robot_mdp.MDP[1][state][action]
        R      += -3 if 6 not in joint_a else -2
        state   = mdp.MDP[0][state][action]

        print(robot_mdp._ext2intAction(action), robot_mdp._ext2intState(robot_mdp.MDP[2][state]), R)
        # Assume that computed policy only selects drop action (5) in the appropriate pos
        if 5 in joint_a:
            break

    return R, state


def showSolution(policy, initial_pos, GIF_filename=None):
    ''' Show pick & place solution '''
    init_state = robot_mdp._int2extState(initial_pos, [0,0])
    next_state = init_state
    robot_mdp.reset(next_state)
    done = False
    while done == False:
        # Show pick&place solution
        robot_mdp.render()
        time.sleep(0.1)
        mapped_state = robot_mdp.MDP[3][next_state]
        action = policy[ mapped_state ]
        print(robot_mdp._ext2intAction(action))
        #next_state, reward, done, info = robot_mdp.step(action)
        next_mapped_state = robot_mdp.MDP[0][mapped_state][action]
        reward            = robot_mdp.MDP[1][mapped_state][action]
        next_state        = robot_mdp.MDP[2][next_mapped_state]
        robot_mdp.reset(next_state)



if __name__ == "__main__":

    # YuMi robot
    from robot_YuMi import Robot_YuMi

    robot = Robot_YuMi()

    # Pieces configuration
    # ... for YuMi
    pieces = [{'start' : [-350, 450, 0],  # Piece 1
               'end'   : [ 350, 400, 0],
              },                     
#              {'start' : [-350, 300, 0],  # Piece 2
#               'end'   : [ 150, 300, 0],
#              },                     
#              {'start' : [-150, 450, 0],  # Piece 3
#               'end'   : [ 250, 250, 0],
#              },                     
              {'start' : [-50,  250, 0],  # Piece 4
               'end'   : [ 250, 500, 0],
              }]

    robot_mdp = mdp_hrl_generator(robot, pieces)

    for i in range(12):

        init_pos   = [[5,0],[6,1]]
        print('Init pos ', init_pos)
        print('Pieces Location ', robot_mdp.piecesLocation)

        # Get MDP template
        if not robot_mdp.load('MDP.bin'):
            robot_mdp.generate()
            robot_mdp.save('MDP.bin')

        # Update it with pieces info
        robot_mdp.setPieces(pieces)
        robot_mdp.update()

        # Solve it
        solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
        policy = solver.solve()

        #state = robot_mdp._int2extState(init_pos, [0,1])
        #state = robot_mdp.MDP[3][state]
        state = np.random.randint(len(robot_mdp.MDP[0]))

        print(getReturn(robot_mdp, policy, state))

    # Show solution
    #showSolution(policy, init_pos)
