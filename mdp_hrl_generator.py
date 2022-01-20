# BUG!!! [[8,0], [3,3]] la alcanza, pero ninguna de las adyacents y e queda bloqeuado
# Not sure if this island can be considered a bug
# Many low MPDs not solved


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

import sys

from env_hrl import env_hrl
from mdp_solver import mdp_solver
import numpy as np
import pickle
import time
import timeit
#import copy

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

        #print('Total states:', self.nS)

        # Filter out invalid states
        valid_states = [s for s in range(self.nS) if self._isStateValid(s)]
        valid_nS = len(valid_states)
        #print('Valid states:', valid_nS)

        states_idx = {}
        for i, item in enumerate(valid_states):
            states_idx[item] = i

        #print("Computing MDP ...")
        # Note: MDP is very large. Use numpy
        mdp_s = np.zeros((valid_nS, self.nA), dtype=np.int32) 
        mdp_r = np.zeros((valid_nS, self.nA), dtype=np.int16)
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

        arm_to_piece = [pos_pieces[0]!=[], pos_pieces[2]!=[]] # Indication (per arm) that it needs to process a piece or not

        # Update all states containing armsStatus [0,2] or [2,0] if only one arm is assigned to a piece
        # The case [2,2] is already taken into account during MDP template generation
        if arm_to_piece != [True, True]:
            #print("SOLO UN BRAZO")
            for mapped_state,state in enumerate(self.MDP[2]):
                _, armsStatus = self._ext2intState(state)
                if ((arm_to_piece[0] == False and armsStatus[1] == 2) or
                    (arm_to_piece[1] == False and armsStatus[0] == 2)):
                    for action in range(48):
                        self.MDP[0][mapped_state][action] = mapped_state
                        self.MDP[1][mapped_state][action] = 0
            
        #print('POS_PIECES ', pos_pieces)
        for p, pos in enumerate(pos_pieces):
            #print('{', pos)
            if pos == []:
                #print('No piece', p)
                continue

            #print('Pos ', pos)
            for x in range(self.M):
                for y in range(self.N):
                    for c_any in range(3):

                        pos_any = [x,y]
                        #print('Pos ', pos, pos_any)

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
                                    c_fix = 2
                                elif p == 2:
                                    a2 = 4
                                    a1 = action_any
                                    c_fix = 1
                                elif p == 3:
                                    a2 = 5
                                    a1 = action_any
                                    c_fix = 2

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
                                    if action_any == 4 or action_any == 5:
                                        if (p<2  and arm_to_piece[1]==False) or (p>=2 and arm_to_piece[0]==False):
                                            # Pick and drop actions are not allowed for the arm not assigned to any piece
                                            continue

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
                                            c_any_next = 2 # piece dropped off
                                        else:
                                            continue


                                # update next_state and reward
                                if p<2:
                                    armsPos_next = [pos, pos_any_next]
                                    armsStatus_next = [c_fix, c_any_next]
                                else:
                                    armsPos_next = [pos_any_next, pos]
                                    armsStatus_next = [c_any_next, c_fix]

                                # Compute reward
                                # TODO: review
                                #if armsPos_next == [pos_pieces[1], pos_pieces[3]]:
                                #if (np.sum(armsStatus_next) == 0) and (armsPos_next[0] == pos_pieces[1]) and (armsPos_next[1] == pos_pieces[3]): reward = 100
                                #if (np.sum(armsStatus_next) == 0) and (armsPos_next[0] == pos_pieces[0]) and (armsPos_next[1] == pos_pieces[2]):
                                #if (a1 == 5 and (armsPos_next[0] == pos_pieces[1]) ) or (a2 == 5 and (armsPos_next[1] == pos_pieces[3]) ):
                                #if armsStatus_next == [2,2]:
                                if (((arm_to_piece[0]==False) or armsStatus_next[0]==2) and
                                    ((arm_to_piece[1]==False) or armsStatus_next[1]==2)):
                                    reward = 10000
                                    #print('PREMIO ', armsPos, armsPos_next)
                                #if self._isGoalMet():  reward = 100 # Goal reached
                                elif action_any == 6:  
                                    reward = -1 # only one arm moving
                                else:
                                    reward = -1 # both arms moving

                                state = self._int2extState(armsPos_next, armsStatus_next)
                                if not self._isStateValid(state):
                                    #print([pos, self.piecesLocation['end'][1]])
                                    #print(a1, a2)
                                    continue

                                # Update next_state/reward
                                self.MDP[0][idx][a1*7+a2] = states_idx[state]
                                self.MDP[1][idx][a1*7+a2] = reward
                                #if pos_pieces[2] == []:
                                #    print(a1,a2, armsPos, armsStatus, reward)


def runLowMDP(policy, mapped_state):
    global num_steps

    done = False
    #print("NEXT")
    while done == False:
        num_steps += 1

        # Show pick&place solution
        #state             = robot_mdp.MDP[2][mapped_state]
        #armsGridPos, armsStatus = robot_mdp._ext2intState(state)
        #print("sds", armsGridPos, armsStatus)

        #mapped_state = robot_mdp.MDP[3][state]
        action = policy[ mapped_state ]
        #print(robot_mdp._ext2intAction(action))
        #next_state, reward, done, info = robot_mdp.step(action)
        next_mapped_state = robot_mdp.MDP[0][mapped_state][action]
        reward            = robot_mdp.MDP[1][mapped_state][action]
        #state             = robot_mdp.MDP[2][next_mapped_state]
        mapped_state      = next_mapped_state

        #TODO: falta condicion de salida (brazo en pieza)
        # Assume that computed policy only selects drop action (5) in the appropriate pos
        joint_a = robot_mdp._ext2intAction(action)
        #print("sds", joint_a)
        state = robot_mdp.MDP[2][mapped_state]
        armsGridPos, armsStatus = robot_mdp._ext2intState(state)
        print(joint_a, armsGridPos, armsStatus)
        if 5 in joint_a:
            pieces_done = [a==5 for a in joint_a]
            break

    # Previous piece done? Force armsStatus to 0
    armsStatus = [ st if st!=2 else 0 for st in armsStatus ]
    state = robot_mdp._int2extState(armsGridPos, armsStatus)
    next_mapped_state = robot_mdp.MDP[3][state]

    return next_mapped_state
    
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
        #print(robot_mdp._ext2intAction(action))
        #next_state, reward, done, info = robot_mdp.step(action)
        next_mapped_state = robot_mdp.MDP[0][mapped_state][action]
        reward            = robot_mdp.MDP[1][mapped_state][action]
        next_state        = robot_mdp.MDP[2][next_mapped_state]
        robot_mdp.reset(next_state)


def getReturn(mdp, policy, state):
    ''' Given the MDP and the computed policy, get the Return (cumulative reward)
        until any of the arms complete its task (pick and place its piece) '''

    R = 0
    pieces_done = [False, False]

    #print(robot_mdp.piecesLocation)

    while True:
        action  = policy[state]
        joint_a = robot_mdp._ext2intAction(action)
        #R      += robot_mdp.MDP[1][state][action]
        R      += -1 if 6 not in joint_a else -1
        #print(mdp)
        #print(joint_a)
        state   = mdp[0][state][action]

        #print(joint_a, robot_mdp._ext2intAction(action), robot_mdp._ext2intState(robot_mdp.MDP[2][state]), R)
        # Assume that computed policy only selects drop action (5) in the appropriate pos
        if 5 in joint_a:
            pieces_done = [a==5 for a in joint_a]
            break

        # Workaround: sometimes, there is no solution (so endless loop)
        if R < -1000:
            #print('RETURN HANG')
            pieces_done = [0,0]
            break

    return R, state, pieces_done


def _updateMDP(prev, p1, p2, num_pieces, state, R):
    global MDP

    if len(MDP[0]) <= prev: # entry already present
        #print('B')
        l = (num_pieces + 1) ** 2
        MDP[0].append([prev for n in range(l)])
        MDP[1].append([-100 for n in range(l)])
    action = (num_pieces + 1)*p1 + p2
    #print(prev, action, MDP[0])
    MDP[0][prev][action] = state
    MDP[1][prev][action] = R
    #print(prev, action, MDP[0])

    return

def getMDPPolicy(num_pieces, p1, p2):
    global robot_mdp
    global pieces
    global MDP_cache
    global solver
    
    #print("policy")
    # Cache (dont repeat computations)
    idx = p1*(num_pieces+1) + p2
    if MDP_cache[idx] != None:
        #print(MDP_cache[0])
        return MDP_cache[idx][0], MDP_cache[idx][1]

    # Choose pieces
    # TODO: workarround
    if p1 == 0:
        two_pieces = [{}, pieces[p2-1]]
    elif p2 == 0:
        two_pieces = [pieces[p1-1], {}]
    else:
        two_pieces = [pieces[p1-1], pieces[p2-1]]
    #print('INPUTTTTTTTT', two_pieces)

    #print(p1, p2)
    #print(pieces)
    #print(two_pieces)
    # Get MDP template
    #if not robot_mdp.load('MDP.bin'):
    #    robot_mdp.generate()
    #    robot_mdp.save('MDP.bin')
    #robot_mdp.MDP = MDP_template[:]
    robot_mdp.load('MDP.bin')
    #robot_mdp.MDP = copy.deepcopy(MDP_template)

    # Update it with pieces info
    robot_mdp.setPiecesLocation(two_pieces)
    robot_mdp.update()
    #print('PP')

    # Solve it
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
    #start = timeit.default_timer()
    policy = solver.solve()
    #end = timeit.default_timer()
    #print(end - start)

    # Update cache
    MDP_cache[idx] = [robot_mdp.MDP[:], policy]
    #print(len(MDP_cache[idx][0]))

    return robot_mdp.MDP[:], policy 


def generateL0MDP(prev, p1, p2, mapped_state, piecesList, total_R, path):

    global this
    global MDP
    global best_total_R
    global best_path
    global path_NA

    if p1 == 0 and p2 == 0:
       print("WTF!!!")
       sys.exit()

    if [p1, p2] in path_NA:
       # This subproblem is known to be non-solveable. Do not compute it again
       # it seems not to optimize nothing. This subMDP was already solved previously and it is the higher computation phase.
       #print("KNOWN not SOLVEABLE ",[p1,p2], path_NA)
       return

    prefix_tab = '  '*len(path)
    #print(prefix_tab + 'generateL0MDP', p1,p2)
    this += 1
    path.append([p1,p2])


    #print("P")
    # Get the appropriate lower layer MDP
    mdp, policy = getMDPPolicy(num_pieces, p1, p2)

    # Run it
    #print(robot_mdp.piecesLocation)
    #print(p1,p2)
    R, mapped_state, pieces_done = getReturn(mdp, policy, mapped_state)
    total_R += R
    #print(R, pieces_done, p1, p2, path)
    #print(prefix_tab, pieces_done)

    if pieces_done == [0,0]:
        #print(prefix_tab + 'PRUNED - path not found')
        path_NA.append([p1,p2])
        return

    if (((pieces_done[0] == False) and (p1 != 0)) or
        ((pieces_done[1] == False) and (p2 != 0))):
        # Hang... (endless loop) No new piece solved
        return

    if total_R <= best_total_R:
        # Prune this path...it is not to get better reward
        #print('PRUNED - Worse path', total_R, best_total_R)
        return
    # Update state - If an arm is done with the piece, reset its status (armStatus: 2->0)
    # mapped_state -> state -> internal state -> fix internal state -> state -> mapped_state
    state = robot_mdp.MDP[2][mapped_state]
    armsGridPos, armsStatus = robot_mdp._ext2intState(state)
    next_armsStatus = [ st if st!=2 else 0 for st in armsStatus ] # Reset arm done
    #print(armsStatus, next_armsStatus)
    state = robot_mdp._int2extState(armsGridPos, next_armsStatus)
    mapped_state = robot_mdp.MDP[3][state]


    #if [p1,p2] == [0,1]:
    #    sys.exit()
    #print(f"{prev} - {this} Pieces {p1}, {p2}, R {R}, List {piecesList}, Done {pieces_done}")

    # Next (p1, p2) selection
    # - If an arm is processing a piece, keep with this piece. Otherwise, take a new one from pending ones or take no piece.
    #
    # First, update the unprocessed-pieces list (example: [0.1,2,3,4] -> [0,1,0,3,4])
    for p in [p1, p2]:
        piecesList[p] = 0

    # Second, select potential next pieces for each arm
    p_list = []
    for i, (p, p_done) in enumerate(zip([p1,p2], pieces_done)):
        if p_done or p==0:
            # Choose a new piece (or o piece)
            p_list.append(set(piecesList))
        else:
            # Stick with the same piece
            p_list.append([p])

    # Third. Stop if done
    if (sum(p_list[0]) + sum(p_list[1])) == 0:
        #print(prefix_tab + 'BRANCH DONE', total_R)
        if total_R > best_total_R:
           best_total_R = total_R
           best_path = path
           #print('REWARD ', best_total_R, path)
        return

    #print(prefix_tab, p_list)
    # Last, Choose one piece (or none) per arm
    for p1 in p_list[0]:
        for p2 in p_list[1]:
            if p1 != p2:
                generateL0MDP(prev, p1, p2, mapped_state, piecesList[:], total_R, path[:])
    

#    # Update piecesList
#    if p1 > 0:
#        piecesList[p1-1] = 0
#    if p2 > 0:
#        piecesList[p2-1] = 0
#
#    # Potential next selections
#    pMap = [[p1], [p2]]
#    for i, (p, p_done) in enumerate(zip([p1,p2], pieces_done)):
#        if p_done:
#            # ... remaining-pieces list (0 if no pieces)
#            tmp = [t for t in piecesList if t != 0]
#            pMap[i] = tmp if tmp != [] else [0]
#
#    #print(pMap)
#    if pMap[0] == pMap[1] and len(pMap[0]) == 1:
#        if pMap[0][0] == 0:
#            # All pieces done
#            num_pieces = len(piecesList)
#            _updateMDP(prev, p1, p2, num_pieces, this, R)
#            # Add terminal state
#            l = (num_pieces + 1) ** 2
#            MDP[0].append([this for n in range(l)])
#            MDP[1].append([0 for n in range(l)])
#            #print(f'Total R: {total_R}')
#            if total_R > best_total_R:
#               best_total_R = total_R
#               best_path = path
#               print('REWARD ', best_total_R, path)
#
#            return
#        else:
#            # Remaining the same piece for both robots
#            pMap[0].append(0)
#            pMap[1].append(0)
#
#    _updateMDP(prev, p1, p2, len(piecesList), this, R)
#
#    # Next state
#    prev = this
#    for p1 in pMap[0]:
#        for p2 in pMap[1]:
#            if p1!=p2:
#                #print(f"BRANCH {prev} {p1} {p2}")
#                generateL0MDP(prev, p1, p2, mapped_state, piecesList[:], total_R, path[:])

def GenerateAndSolveMDP(pieces, armsGridPos):

    state = robot_mdp._int2extState(armsGridPos, [0,0])
    mapped_state = robot_mdp.MDP[3][state]
    num_pieces = len(pieces)
    piecesList = [i for i in range(num_pieces+1)] # [0,1,2,3,4]
    #MDP_cache = [None for _ in range((num_pieces+1)**2)]
    #MDP = [[], []]

    for p1 in piecesList:
        for p2 in piecesList:
            if p1 != p2:
                generateL0MDP(0, p1, p2, mapped_state, piecesList[:], 0, [])

    state = robot_mdp._int2extState(armsGridPos, [0,0])
    mapped_state = robot_mdp.MDP[3][state]

    #print('C', best_path, best_total_R)
    for p1,p2 in best_path:

        #print(p1, p2)

        # Get the appropriate lower layer MDP
        mdp, policy = getMDPPolicy(num_pieces, p1, p2)

        # Execute low level MDP
        robot_mdp.MDP = mdp
        mapped_state = runLowMDP(policy, mapped_state)
    #print(f"Solution in {num_steps}")

if __name__ == "__main__":

    # YuMi robot
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
              {'start' : [-50,  250, 0],  # Piece 4
               'end'   : [ 250, 500, 0],
              }]

    pieces = [{'start' : [-350, 450, 0],  # Piece 1
               'end'   : [ 250, 250, 0],
              },                     
              {'start' : [-350, 300, 0],  # Piece 2
               'end'   : [ 150, 300, 0],
              },                     
              {'start' : [-50,  350, 0],  # Piece 3
               'end'   : [ 350, 400, 0],
              },                     
              {'start' : [-150, 450, 0],  # Piece 4
               'end'   : [ 250, 500, 0],
              }]
    pieces = [{'start' : [-250, 500, 0],  # Piece 1
               'end'   : [ 250, 200, 0],
              },                     
              {'start' : [-350, 200, 0],  # Piece 2
               'end'   : [ 150, 300, 0],
              },                     
              {'start' : [-50,  200, 0],  # Piece 3
               'end'   : [ 350, 400, 0],
              },                     
              {'start' : [-150, 400, 0],  # Piece 4
               'end'   : [ 50, 500, 0],
              }]
    pieces = [{'start' : [-150, 200, 0],  # Piece 1
               'end'   : [ 250, 200, 0],
              },                     
              {'start' : [-250, 200, 0],  # Piece 2
               'end'   : [ 150, 300, 0],
              },                     
              {'start' : [-50,  200, 0],  # Piece 3
               'end'   : [ 350, 400, 0],
              },                     
              {'start' : [-250, 400, 0],  # Piece 4
               'end'   : [ 50, 500, 0],
              }]
#    pieces_fake = [{'start' : [-350, 450, 0],  # Piece 1
#               'end'   : [ 350, 400, 0],
#              },                     
#              {'start' : [-350, 300, 0],  # Piece 2
#               'end'   : [ 150, 300, 0],
#              },                     
#              {'start' : [-150, 450, 0],  # Piece 3
#               'end'   : [ 250, 250, 0],
#              },                     
#              {'start' : [-50,  250, 0],  # Piece 4
#               'end'   : [ 250, 500, 0],
#              }]

    robot_mdp = mdp_hrl_generator(robot, pieces)

    robot_mdp.setPiecesLocation(pieces)
    #print(robot_mdp.piecesLocation)

    # Get MDP template
    if not robot_mdp.load('MDP.bin'):
        robot_mdp.generate()
        robot_mdp.save('MDP.bin')

    #MDP_template = robot_mdp.MDP[:]
    #MDP_template = copy.deepcopy(robot_mdp.MDP)

    # Other dirty thing to check: annotate unsolveable cases and do not repeat computation
    path_NA = []

    # Test the solution for several pieces configurations and several arms init locations
    #
    np.random.seed(5)
    print("ArmsGridPos;Pieces location;Num steps")
    for j in range(5):
        pieces = [{'start' : [-50 - 100*np.random.randint(5), 200 + 100*np.random.randint(5), 0],  # Piece 1
                   'end'   : [ 50 + 100*np.random.randint(5), 200 + 100*np.random.randint(5), 0],
                  } for _ in range(4)] # 4 pieces
        ## Trick to add pieces (more random locations would end up in some unreachable piece for both arms)
        #for kk in range(5):
        #   pieces.append(pieces[kk].copy())
        pieces_str = pieces

        for _ in range(5):
            armsGridPos = [[np.random.randint(5),np.random.randint(5)], [np.random.randint(5,10),np.random.randint(5)]]

            #if j<2: continue

            num_steps = 0
            this = 0
            best_path = []
            best_total_R = -2000
            num_pieces = len(pieces)
            MDP_cache = [None for _ in range((num_pieces+1)**2)]
            MDP = [[], []]

            # check pieces reachability
            #for tt in range(2):
            #    print("ARM {}".format(tt))
            #    for piece in pieces:
            #        print("  PIECE {}".format(piece))
            #        for phase in ['start', 'end']:
            #            xy  = robot_mdp._nearest2DTo(piece[phase], robot_mdp.robot.location)
            #            print(xy)
            #            idx = robot_mdp._xy2idx(xy)
            #            valid = robot_mdp.robot.reachable[tt][idx]==1
            #            #valid = valid or robot_mdp.robot.checkCollision()
            #            print("    {} : {}".format(phase, valid))

            #################
            # HRL loe level MDP computation broken!!!!
            #################
            #mdp, policy = getMDPPolicy(num_pieces, 2, 3)

            #state = robot_mdp._int2extState(armsGridPos, [0,0])
            #mapped_state = robot_mdp.MDP[3][state]
            #R, state, pieces_done = getReturn(mdp, policy, mapped_state)
#            GenerateAndSolveMDP(pieces, armsGridPos)
#            import sys
#            sys.exit()
            #################
            # END
            #################
            try:
                GenerateAndSolveMDP(pieces, armsGridPos)
            except:
                # There is a chance that pieces configuration/arms init locations results in an invalid case
                print('except')
                pass
            print(armsGridPos, ";", pieces_str, ";", num_steps)
            import sys
            sys.exit()
            pieces_str = "IDEM" # Cleaner than repeating the whole pieces location
#    sys.exit()

#    best_path = []
#    best_total_R = -200
#    this = 0
#    #armsInitPos = [[1,2],[3,4]]
#    armsInitPos = [[1,4], [8,4]]
#    armsInitStatus = [0,0]
#    armsGridPos = armsInitPos[:]
#    armsStatus  = armsInitStatus[:]
#    state = robot_mdp._int2extState(armsGridPos, armsStatus)
#    mapped_state = robot_mdp.MDP[3][state]
#    piecesList = [i+1 for i in range(len(pieces))]
#    num_pieces = len(piecesList)
#    MDP_cache = [None for _ in range((num_pieces+1)**2)]
#    MDP = [[], []]
#
#    #p1,p2 = 0,1
#    #mdp, policy = getMDPPolicy(len(piecesList), p1, p2)
#    #R, state, pieces_done = getReturn(mdp, policy, mapped_state)
#    #sys.exit()
#
#    for p1 in piecesList:
#        for p2 in piecesList:
#            if p1 != p2:
#                #print('New')
#                generateL0MDP(0, p1, p2, mapped_state, piecesList[:], 0, [])
##                #print(MDP[0])
##                #print(MDP[1])
##                if p1 ==2:
##                    sys.exit()
#    #print(best_total_R)
#    #print(best_path)
#
#    #sys.exit()
#
#    # Show solution
#    armsGridPos = armsInitPos[:]
#    armsStatus  = armsInitStatus[:]
#    state = robot_mdp._int2extState(armsGridPos, armsStatus)
#    mapped_state = robot_mdp.MDP[3][state]
#
#
#    print(best_path, best_total_R)
#    #sys.exit()
#    num_steps = 0
#    for p1,p2 in best_path:
#
#        print(p1, p2)
#
#        # Get the appropriate lower layer MDP
#        mdp, policy = getMDPPolicy(num_pieces, p1, p2)
#
#        # Execute low level MDP
#        robot_mdp.MDP = mdp
#        mapped_state = runLowMDP(policy, mapped_state)
#    print(f"Solution in {num_steps}")

#####################################################################

#    for i in range(12):
#
#        init_pos   = [[5,0],[6,1]]
#        #print('Init pos ', init_pos)
#        #print('Pieces Location ', robot_mdp.piecesLocation)
#
#        # Get MDP template
#        if not robot_mdp.load('MDP.bin'):
#            robot_mdp.generate()
#            robot_mdp.save('MDP.bin')
#
#        # Update it with pieces_fake info
#        robot_mdp.setPiecesLocation(pieces_fake)
#        #robot_mdp.setPiecesRobotLocation(pieces_fake)
#        robot_mdp.update()
#
#        # Solve it
#        solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
#        policy = solver.solve()
#
#        #state = robot_mdp._int2extState(init_pos, [0,1])
#        #state = robot_mdp.MDP[3][state]
#        state = np.random.randint(len(robot_mdp.MDP[0]))
#
#        #print(getReturn(robot_mdp, policy, state))

#     Show solution
#     showSolution(policy, init_pos)


# HELPERS
# x goes from 0 to 9 (450 -> -450)
# y goes from 0 to 4 (200 -> 600)
# pos = [x, y]
# idx = x + 10*y
# [p1, p2] -> p1 es sbrazo izquierda en la excel (que curiosamente esta en la columna de la derecha)
#
#

