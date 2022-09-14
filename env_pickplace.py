# pick_pos>0 y pieces_status == 0 no es valid
import time
import random
import numpy as np
import pickle
  # NEED TO INSTALL THIS PACKAGE
  #import matplotlib.pyplot as plt
  #from mpl_toolkits import mplot3d
  #from matplotlib.backend_bases import MouseButton
  # NEED TO INSTALL TINYIK (and add a change if I remember how)
  #from robot_2A2L import Robot_2A2L
from robot_YuMi import Robot_YuMi
import code

# Debug functionality
#def readfunc(a):
#    return input()
#code.interact(readfunc=readfunc, local=globals()

# MDP
# Consider 2D (z=0) moves for Arm EEs
# - exception: pick and place punctual operations (not needed a 3D map anyway)
#
# STATES
# ------
# EE arm pos (x, y, z)  <- N*M individual pos states 
# Joint arms pos states <- (N*M)^2
# K pieces              <- (2^K) pieces states
# EE arm status (free or carrying piece i) <- K+1
# (There are two arms: EE1 and EE2)
#
# MDP state size = ((N*M)^2)* ((K+1)^2) * (2^K) 
# 
# Example 1:
# - (10x10 grid, two arms and 2 pieces) -> N=10, M=10, K=2
# - MDP state size = 10000*4*9 = 360 000 
# 
# Example 2:
# - (10x10 grid, two arms and 4 pieces) -> N=10, M=10, K=4
# - MDP state size = 10000*16*25 = 4 million
# 
# ACTIONS
# -------
# Individual arm actions (up/down/left/right/pick/place/stay)
# Joint arm actions: 7^2 - 1 (stay for both arms) = 48
#
# REWARDS
# -------
# -2   : move both arms
# -1.9 : move only one arm (preferred to both arms)
# -10  : collision or not reachable,
# +100 : goal
#
# MDP
# ---
# MDP size = 4 million * 48 = 192 million (next state, reward) pairs

# Ideas:
# - Remove invalid MDP states (keeping a table to map index to real state scalar)
# - Remove invalid actions per state? maybe pick/place in many states


# TODO:
# - Error logica. No podemos forzar a un brazo a no moverse una vez alcanzado su destino.
#   Es preferible que no se mueva, pero es posible que si no se mueve bloquee el camino al 
#   otro brazo y no se pueda completar la tarea.
# - Si discountFactor == 1, hay muchos estados del Value Function que no converjan nunca.
#   Por ej. un estado que define que quedan dos piezas, una llevada ya por cada brazo, y sus 
#   posiciones finales son tales que los brazos colisionan (luego nunca alcanzan el estado 
#   final que es el que garantiza convergencia... (reward en el estado final es 0)). Esto 
#   viene apoyado en el punto anterior.... un brazo se bloquea al dejar su ultima pieza

class env_pickplace:
    ''' Pick&Place environment
    
        The interface is similar to the openAI gym one, but less elaborate
        Bare minimum example:

            from envYumi import EnvYuMi

            env = EnvYuMi()
            env.reset()
            for _ in range(10):
                env.render()
                env.step(env.action_space.sample()) # take a random action
            env.close()
    '''
    def __init__(self, robot, pieces_cfg):

        #
        # Environment definition (YuMi + pieces)
        #
        self.robot     = robot
        self.piecesCfg = pieces_cfg

        # 'robot' object describes a two-arm robot with n degrees of freedom.
        # 
        # Although the robot configuration seems to live in the continuos domain
        # (any angle value for each joint), since the manufacturer defines some 
        # constraints (angle range and resolution), you could consider the robot
        # living in the discrete domain. Anyway, there is such a huge number of
        # states (YuMi robot: 14^discret_values_per_joint), that it does not 
        # matter in which domain you consider the robot lives.
        #
        # Let's assume some simplifications to reduce the problem complexity:
        # - Do not work with robot configuration (angles) but with its EE 
        #   position. This will require to precompute robot configuration 
        #   (angles) for all EE potential positions.
        #   This way, no matter the number of degrees of freedom of the robot,
        #   but the EE positions
        # - Discretize state space: N*M grid (EndEffector 2D positions).
        #   Pick&Place operations are considered atomic operations (ignored the
        #   up/down movement in the state space)
        #   - You can consider some extra states for the pick&place actions in 
        #     order to match the time with any move in 2D.
        # - Discretize action space: up/down/left/rigth/pick/place/stay 
        #   (movements constrained to reach adjacent grid cells)
        # - Do not base MDP reward on arms movement interval time. Simplify
        #   considering '-1' reward per each move.
        #
        # Note: This will generate a very discretized trayectory. It will 
        #       require posterior processing to smooth it.
         
         
        # State space:
        # - Robot moves in a 2D M*N grid + pick/drop operations in vertical direction
        # - Each state is internally represented by:
        #   - arms position (x,y) and pickPosition
        #   - arms status (arm carrying the piece i or none)
        #   - pieces status (piece pending to be picke up)
        #
        # - Each state is externally represented by a single scalar (see
        #   _int2extState() function)
        #
        self.M, self.N = self.robot.M, self.robot.N
        self.K = len(self.piecesCfg)
        self.T = 1 # Number of available intermediate positions (limited up to 1 by current implementation)
        self.P = 0 # Number of ´extra´ states representing the pick up or drop off operation
                   #  P = 0      2      4
                   #     .->.   .  .   .  .
                   #            |  |   |  |
                   #             ->    |  |
                   #                    ->
 
        # Initial state (internal representation) 
        self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickStatus = self._getDefaultResetState()

        # Extra intermediate position
        self.T_pos = [3, 3] # arbitrary value (it should be a valid position for both arms)
 
        # State space size
        # Note: instead fo M*N*P (3D), we extend Z axis only for K cells (where piece can be picked or dropeed). This is just to reduce memory resources. In the end we would end up filtering out those cells where you can not pick/drop a piece
        self.nS = (((self.M * self.N) + (self.K * self.P))**2) * ((2 + self.T)**self.K) * ((self.K + 1)**2)

        # Action space size: 
        self.single_nA = 7
        self.nA = self.single_nA**2 - 1 # all join actions (up/up, up/left, down/up, ...) except stay/stay

        # Pieces locations are also mapped to the same robot 2D grid (even if
        # the real piece position does not match the 2D grid point)
        #
        #                              ____ piece pos (1,1)
        #   (0,0)                     |
        #     .   .   .   .         . v .   .   .
        #     .   .   .   .   -->   . .     .   .
        #     .   .   .   .         .   .   .   .
        #
        # Note: Risk! several pieces might fall in the same grid cell
        # Note: Real pieces location are stored in order to move the robot to
        #       the right positions, instead of using the standard 2D defined
        #       grid.
        #
        self.piecesLocation = { 'start':[], 'end':[] }
        for cfg in self.piecesCfg:
            # find corresponding grid location for each piece
            xy_i = self._nearest2DTo(cfg['start'], self.robot.location)
            xy_j = self._nearest2DTo(cfg['end'],   self.robot.location)

            self.piecesLocation['start'].append(xy_i)
            self.piecesLocation['end'  ].append(xy_j)

            ## update robot grid
            #print(xy_i, self._xy2idx(xy_i), cfg['start'])
            #print(xy_j, self._xy2idx(xy_j), cfg['end'])
            self.robot.updateLocation(self._xy2idx(xy_i), cfg['start'])
            self.robot.updateLocation(self._xy2idx(xy_j), cfg['end'])

        # Pieces current pos
        self.piecesCurrPos = [ cfg for cfg in self.piecesLocation['start'] ]

#        # Get robot (angles) configurations
#        self.gridAng = self.robot.getConfig()
#
        # Open AI gym-style stuff)
        self.action_space      = self.ActionSpace(self.nA)
        self.observation_space = self.ObservationSpace(self.nS)


    #############################################################
    #
    # Helper functions
    #
    #############################################################
    def _getIntState(self):
        return [self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos]

    def _getExtState(self):
        return self._int2extState(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos)

    def _deriveXYfromState(self, armStatus, piecesStatus, pickPos):
        ''' XY arm pos and pickPos info are combined in the MDP: (M*N + K*P), so
            when P>0, XY info is not encoded and need to be derived from the other
            MDP state vars '''

        if armStatus > 0:
            piece = armStatus - 1
        elif pickPos > 0:
            piece = (pickPos - 1) // self.P
        else:
        #    assert False, 'Really? armsStatus is 0'
            # Impossible to get the info
            return [-1,-1]

        pStatus = piecesStatus[piece]
        if pStatus == 0:   # end
            x,y = self.piecesLocation['end'][piece]
        elif pStatus == 1: # init
            x,y = self.piecesLocation['start'][piece]
        elif pStatus == 2: # intermediate 1
            x,y = self.T_pos
        else:              # unexpected
        #    assert False, 'Really? unknonw piece Status'
            x,y = -1,-1

        return [x,y]

    def _int2extState(self, armsPos, armsStatus, piecesStatus, pickPos):
        ''' convert internal state representation to a single scalar '''
        N,M,K,P,T = self.N, self.M, self.K, self.P, self.T

        tmp = 0
        for status in piecesStatus:
            tmp *= (2 + T)
            tmp += status
        state = tmp

        tmp = 0
        for status in armsStatus:
            tmp *= (K + 1)
            tmp += status
        state += tmp * ((2 + T)**K)

        tmp = 0
        for (x,y),p in zip(armsPos, pickPos):
            tmp *= (N*M + K*P)
            if p == 0 or self.P == 0:
                tmp += self._xy2idx([x,y])
            else:
                tmp += N*M-1 + p
        state += tmp * ((2 + T)**K) * ((K+1)**2)

        return state

    def _ext2intState(self, state):
        ''' convert external state (scalar) to internal state representation '''
        N,M,K,P,T = self.N, self.M, self.K, self.P, self.T

        pieces_status = []
        for _ in range(K):
            tmp = state % (2 + T)
            pieces_status.insert(0, tmp)
            state = (state - tmp) // (2 + T)

        arms_status = []
        for _ in self.armsStatus:
            tmp = state % (K+1)
            arms_status.insert(0, tmp)
            state = (state - tmp) // (K+1)

        arms_pos = [] 
        pick_pos = []
        for arm_st in reversed(arms_status):
            idx = state % (N*M + K*P)
            state = (state - idx) // (N*M + K*P) 
            if idx >= N*M:
                p_pos = idx + 1 - N*M
                arms_pos.insert(0, self._deriveXYfromState(arm_st, pieces_status, p_pos) )
                pick_pos.insert(0, p_pos)
            else:
                arms_pos.insert(0, self._idx2xy(idx))
                pick_pos.insert(0, 0)

        return [arms_pos, arms_status, pieces_status, pick_pos]

    def _ext2intAction(self, action):
        ''' convert external action (scalar) to internal action representation '''
        return [action//self.single_nA, action%self.single_nA]
    
    def _int2extAction(self, joint_action):
        ''' convert internal action representation to a single scalar '''
        action = 0
        for a in joint_action:
            action *= self.single_nA
            action += a

        return action

    def _nearest2DTo(self, item, item_list):
        ''' Find the most similar entry in the item_list (ignore Z) '''
        dist = [ np.sqrt(sum( (a - b)**2 for a, b in zip(item[0:-1], p[0:-1]))) for p in item_list]
        return self._idx2xy(np.argmin(dist))

    def _nearestTo(self, item, item_list):
        ''' Find the most similar entry in the item_list '''
        dist = [ np.sqrt(sum( (a - b)**2 for a, b in zip(item, p))) for p in item_list]
        return self._idx2xy(np.argmin(dist))

    def _xy2idx(self, xy):
        ''' xy grid pos to index '''
        # (N-1)*M, (N-1)*M+1, (N-1)*M+2, ... N*M-1    y
        #                                             ^
        #       M,       M+1,       M+2, ... 2M-1     |
        #       0,         1,         2, ...  M-1     o--> x
        x,y = xy
        return x + y*self.M

    def _idx2xy(self, idx):
        ''' xy grid pos to index '''
        return [idx % self.M, idx // self.M]

    def _logic2phy(self, pos):
        ''' convert logical grid cell pos to physical 3D point
            [0,1] -> [-1.35, -0.5, 0]
        '''
        return self.robot.location[ self._xy2idx(pos) ]

    def _phy2logic(self, pos):
        ''' convert physical 3D point to logical grid cell pos
            [-1.05, -0.8, 0] -> [1,2]
        '''
        # Do not compare floats. Find the most similar cell pos
        return self._nearestTo(pos, self.robot.location)

#    def _logic2ang(self, arm_idx, pos):
#        ''' convert logical grid cell pos to robot angles configuration
#            [0,1] -> [2.61814353, 2.11034636]
#        '''
#        return self.gridAng[arm_idx][ self._xy2idx(pos) ]
#
    def _isArmsGridPosValid(self, pos):
        ''' Check if pos is inside the grid boundaries '''
        x,y = pos
        return not ((x >= self.M) or (x < 0) or (y >= self.N) or (y < 0))

    def _isPiecesStatusValid(self, armsStatus, piecesStatus, pickPos):
        ''' Check arms and pieces status coherence '''

        res = True

        # - incoherence: the same piece carried by several arms
        picked = [x for x in armsStatus if x>0]
        if len(picked) != len(list(set(picked))):
            res = False

        # - incoherence: a piece has not been picked up yet, and it is carried by an arm at the same time
        if res == True:
            for piece,pp in zip(armsStatus, pickPos):
                # Note: this check is not valid for pp>0, because armsStatus is 
                #       modified (for convinience) at pp==1 during DROP
                if (piece > 0) and (pp == 0) and (piecesStatus[piece - 1] > 0):
                    res = False
                    break

        # - incoherence: pick position not valid
        if res == True:
            # Both arms picking/dropping the same piece
            if 0 not in pickPos:
                if (((pickPos[0] - 1) // self.P) == ((pickPos[1] - 1) // self.P)):
                    res = False
                
        if res == True:
            for piece, pp in zip(armsStatus, pickPos):
                # Drop operation pick_pos and piece need to match
                if piece>0 and pp>0 and piece != (((pp - 1) // self.P) + 1):
                     # carrying a piece and pick/drop other
                     res = False
                     break

        return res


    def _isPickUpActionValid(self, arm_grid_pos, arm_status, pieces_status, pick_pos):
        ''' Check pick up action coherence in the current state for a specific arm '''

        piece = 0
        valid = False

        if arm_status == 0: # arm empty
            if pick_pos > 0:
                # arm_pos unknown when pick_pos>0
                piece = (pick_pos - 1) // self.P
                if pieces_status[piece] > 0:
                    valid = True
                else:
                    valid = False
            else:
                for i,(piece_pos, piece_status) in enumerate(zip(self.piecesLocation['start'], pieces_status)):
                    if (piece_status == 1 and arm_grid_pos == piece_pos) or \
                       (piece_status == 2 and arm_grid_pos == self.T_pos):
                       piece = i
                       valid = True
                       break

        return valid, piece

    def _isDropOffActionValid(self, arm_grid_pos, arm_status, pick_pos):
        ''' Check drop off action coherence in the current state for a specific arm '''

        piece = 0
        piece_status = 0
        valid = False

        if arm_status > 0:
            piece = arm_status - 1

            if pick_pos > 0:
                assert (piece == (pick_pos-1)//self.P), 'WTF..different piece'
                #TODO review this piece_status = 0
                piece_status = 0
                valid = True
            else:
                if (arm_grid_pos == self.piecesLocation['end'][piece] ):
                    piece_status = 0
                    valid = True
                elif (arm_grid_pos == self.T_pos) and self.T > 0:
                    piece_status = 2
                    valid = True

        return valid, piece, piece_status

    def _isGoalMet(self):
        ''' Is MDP solved? '''
        return ((np.sum(self.piecesStatus) == 0) and (np.sum(self.armsStatus) == 0) and (np.sum(self.pickPos) == 0))

    def _getDefaultResetState(self):
        armsGridPos = [[0,0], [6,4]]  # init 2D grid pos
        armsStatus  = [0, 0]          # init status: carrying no piece
        piecesStatus= [1 for _ in range(self.K)] # init status: No piece has been picked up yet
        pickPos     = [0,0]

        return [armsGridPos, armsStatus, piecesStatus, pickPos]

    def reset(self, state=None):
        ''' Bring environment to the default state '''

        if state == None:
            self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos = self._getDefaultResetState()
        else:
            self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos = self._ext2intState(state)

        self.piecesCurrPos = [ cfg for cfg in self.piecesLocation['start'] ]

    # Move robot part to Robot class
    def render(self):
        ''' Show environment '''
        #TODO: render (_plot) is both robot and environment
        # Update robot configuration
        # Move this render action to Robot
        #pos = [self._logic2ang(i, grid_pos) for i,grid_pos in enumerate(self.armsGridPos)]
        #self.robot.setAngles(pos)

        # Plot robot
        self._plot()

    #def step(self, action):
    #    ''' Take an action. Make partial processing. Use reward data from 
    #        self.MDP to avoid the heavy processing part. This code is a reduced
    #        version of '_step()' function.

    #        The point is to generate and store a MDP containing only reward, 
    #        done and invalid data (all in the same byte) to reduce memory
    #        footprint. Then, next_state data is computed on the fly in each 
    #        function call.
    #    '''
    #    done   = False
    #    info   = ''

    #    state_idx = {}
    #    for i, item in enumerate(self.MDP[2]):
    #        state_idx[item] = i

    #    state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap, self.pickPos)
    #    reward = self.MDP[1][state_idx[state]][action]

    #    # Goal completed. Do nothing (reward is 0)
    #    #
    #    if self._isGoalMet():
    #        reward = 0
    #        done   = True

    #    # Invalid next_state
    #    elif reward <= -5: # TODO: better to define this as a macro
    #        pass

    #    # Valid next_state
    #    else:
    #        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
    #        joint_a = self._ext2intAction(action)

    #        for i,(a, pos) in enumerate(zip(joint_a, self.armsGridPos)):
    #            # move right/left/down/up
    #            if a < 4: 
    #                pos = list(pos + offset_a[a])
    #                self.armsGridPos[i] = pos
    #                piece_idx = self.armsStatus[i]
    #                if piece_idx > 0:
    #                    self.piecesCurrPos[piece_idx-1] = pos
    #            # pick up
    #            elif a == 4:
    #                _, piece_idx = self._isPickUpActionValid(pos, self.armsStatus[i], self.piecesMap)
    #                self.armsStatus[i]  = piece_idx + 1
    #                self.piecesMap     &= ~(1<<piece_idx)
    #            # drop off
    #            elif a == 5:
    #                self.armsStatus[i]  = 0

    #        state = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap, self.pickPos)

    #    return state, reward, done, info

    def _isStateValid(self, state):
        '''  Convert scalar state number to internal MDP state vars representation
             and check if that vars combination is valid 
        '''

        arms_pos, arms_status, pieces_status, pick_pos = self._ext2intState(state)

        valid_state = True

        # Check if arms pose is feasible 
        if valid_state:
            valid_state = self.robot.checkValidLocation(arms_pos)

        # Check for arms collision
        if valid_state:
            valid_state = not self.robot.checkCollision(arms_pos)

        if valid_state:
            valid_state = self._isPiecesStatusValid(arms_status, pieces_status, pick_pos)

        if (state % (self.nS/100)) == 0:
            print(state / (self.nS/100))

        return valid_state

    def _step(self, action, mode=0):
        ''' Given a state "s" and an action "a", returns a state "s'" and a reward "r".

            If the action "a" is wrong (grid boundaries exceeded, robot collision,
            ...) on state "s", the state won't change and the reward will be highly
            negative.

            Note: 'mode' is an internal argument to trick the pick&place actions.
                  When building the MDP to be agnostic of the pieces configuration,
                  pick&place actions are not resolved. Setting 'mode' to 1,
                  leaves a special mark in the reward. This mark will be later
                  used to update the MDP once the specific set of pieces 
                  configuration is known.
        '''

        # First, check if we've already met the goal
        #
        if self._isGoalMet():
            # Do not process the action. By design, any action results in the
            # same goal state with reward 0.
            state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos)
            reward = 0
            done   = True
            info   = ''
            return state, reward, done, info

        # Mode 1 means that pieces position is unknown, so some state-action
        # pairs (ie: pick piece 1) can not be yet resolved.
        #
        if mode == 1:
            joint_a = self._ext2intAction(action)
            for i,a in enumerate(joint_a):
                # Pick/drop
                if ((a == 4) and (self.armsStatus[i] == 0)) or (((a == 5) and (self.armsStatus[i] > 0))):
                   if (self.pickPos[i] > 0) or \
                      (self.pickPos[i] == 0 and self.armsGridPos[i] != self.T_pos): # actually, intermediate position can be processed
                       # Add special mark in the reward function
                       # This state-action pair will be resolved when pieces position is known (later with update() function)
                       state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos)
                       reward = -30
                       done   = False
                       info   = ''
                       return state, reward, done, info

        # Now, let's see what happens when applying the agent action. Wrong 
        # actions will be punished, while the good ones will be reinforced
        # (reward reponse)
        #
        # Wrong action reasons:
        # - Robot exceeds grid boundaries
        # - Incoherent pick up or drop off action (nothing to pick up or drop off)
        # - Robot arms collision
        #
        valid_state     = True
        pick_place_mark = False
        info            = ''
        done            = False

        # Preparing MDP state vars for the next time-step
        next_arms_status     = self.armsStatus[:]                 # copy 1D list
        next_pieces_status   = self.piecesStatus[:]               # copy 1D list
        next_arms_grid_pos   = []
        next_pick_pos        = self.pickPos[:]

        next_pieces_grid_pos = [x[:] for x in self.piecesCurrPos] # copy 2D list
        move_both_arms       = True 

        # helper for adjacent moves
        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up

        #print(self._int2extState(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos))
        joint_a = self._ext2intAction(action)
        for i,(a, pos_arg, arm_status, pick_pos) in enumerate(zip(joint_a, self.armsGridPos, self.armsStatus, self.pickPos)):

            pos = pos_arg[:] # Dont want to directly modify a for-loop argument

            # Move (right/left/down/up)
            if a < 4:
                if pick_pos > 0:
                    valid_state = False # pick/drop operation only allows pick/drop actions
                    break

                pos = list(pos + offset_a[a])
                if not self._isArmsGridPosValid(pos):
                    valid_state = False # Grid boundaries exceeded
                    break

                # update piece pos (helper for plotting)
                # TODO: I think this can be derived from state and no need for
                # extra vars for plotting
                if arm_status > 0:
                    next_pieces_grid_pos[arm_status - 1] = pos

            # pick or place
            #
            elif a == 4 or a == 5:

                # Remind that pick_pos is 0..K*P (piece 1: 1..P, pieces 2: P+1..2P, ...)
                if pick_pos > 0: reduced_pick_pos = ((pick_pos - 1) % self.P) + 1
                else:            reduced_pick_pos = 0

                # pick up
                if a == 4:
                    valid_state, piece_idx = self._isPickUpActionValid(pos, arm_status, self.piecesStatus, pick_pos)
                    if valid_state:
                        if reduced_pick_pos >= self.P:
                            next_pick_pos[i] = 0
                            next_arms_status[i] = piece_idx + 1
                            next_pieces_status[piece_idx] = 0 # piece picked up
                        else:
                            # creo que ambas lineas son lo mismo
                            next_pick_pos[i] = reduced_pick_pos + 1 + piece_idx*self.P

### pos should be unchanged
                        if pick_pos > 0:
                            #print(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos)
                            #print(pos , self.piecesLocation['start'][(pick_pos-1)//self.P])
                            #assert (pos == self.piecesLocation['start'][(pick_pos-1)//self.P]), 'Mmm why did it change?'
                            pos = self.piecesLocation['start'][(pick_pos-1)//self.P]
                            pass
                    else:
                        break # Nothing to pick up

                # drop off
                elif a == 5:
                    valid_state, piece_idx, piece_status = self._isDropOffActionValid(pos, arm_status, pick_pos)
                    if valid_state:
                        if reduced_pick_pos >= self.P:
                            next_pick_pos[i] = 0
                            next_arms_status[i] = 0
                            if reduced_pick_pos == 0:
                                next_pieces_status[piece_idx] = piece_status
                        else:
                            # creo que ambas lineas son lo mismo
                            next_pick_pos[i] = reduced_pick_pos + 1 + self.P*piece_idx
                            # For convinience, we anticipate the piece status update
                            if reduced_pick_pos == 0:
                                next_pieces_status[piece_idx] = piece_status

###                        # Fix location
                        if pick_pos > 0:
                            #TODO esto me huele a que con mode 0 o 1 debemos hacer diferente chequeo
                            #assert (pos == self.piecesLocation['start'][(pick_pos-1)//self.P]), 'Mmm why did it change in drop?'
                            pos = self.piecesLocation['end'][(pick_pos-1)//self.P]
                            pass
                    else:
                        break # Nothing to drop off
            # stay
            else:
                move_both_arms  = False # One arm stays still
                #TODO: Tbis should be OK
                #if pick_pos>0:
                #    valid_state = False
                #    break

            next_arms_grid_pos.append(pos)

        if valid_state:
            valid_state = self.robot.checkValidLocation(next_arms_grid_pos)

        if valid_state:
            valid_state = not self.robot.checkCollision(next_arms_grid_pos)

        if valid_state:
            valid_state = self._isPiecesStatusValid(next_arms_status, next_pieces_status, next_pick_pos)

        if valid_state:
            if pick_place_mark:
                # special reward mark (state unchanged)
                reward = -30
            else:
                # No need to copy, just point to the next_xxx ones
                self.armsStatus    = next_arms_status
                self.piecesStatus  = next_pieces_status
                self.armsGridPos   = next_arms_grid_pos
                self.pickPos       = next_pick_pos
                self.piecesCurrPos = next_pieces_grid_pos

                if self._isGoalMet():
                    reward = 10000
                    done   = True
                else:
                    # Different reward when moving one or both arms helps to:
                    # - prefer moving both arms at once (-2) than separately (-3.8)
                    # - prefer moving only one arm when the other already got 
                    #   its goal (-1.9 better than -2)
                    #if joint_a[1] != 6:         reward = -5
                    #elif move_both_arms == False: reward = -1 # 1 arm
                    if move_both_arms == False: reward = -1 # 1 arm
                    else:                       reward = -1 # 2 arms
        else:
            # The same reward (punishment) for any unwanted action
            reward = -20

        # Build response
        observation = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesStatus, self.pickPos)


        return observation, reward, done, info

    def close(self):
        ''' Close environment '''
        pass

    # Move robot part to Robot class
    def _plot(self):

        plt3d = False

        # Draw
        try:
            self.ax.clear()
        except AttributeError: 
            if plt3d:
                self.ax = plt.axes(projection='3d')
                self.fig = plt.gcf()
            else:     
                self.ax = plt.axes()
                self.ax.set_aspect('equal')
                self.fig = plt.gcf()

        #plt.axis([-3,3,-3,3]) # TODO: no poner a fuego (usar area del grid como referencia)
        plt.axis([-600,600,50,750]) # TODO: no poner a fuego (usar area del grid como referencia)

        # Robot
        phyPos = [self._logic2phy(pos) for pos in self.armsGridPos]
        self.robot.plot(phyPos, self.ax, plt3d)

        # Pieces
        partsColor ='bgrcmykbgrcmyk'
        now, end = [[],[],[],[]], [[],[],[],[]] # x,y,z,color
        for i, (grid_pos, part) in enumerate(zip(self.piecesCurrPos, self.piecesCfg)):
            pos = list(self._logic2phy(grid_pos)[:])
            pos.append(partsColor[i])
            for j,data in enumerate(pos):
                now[j].append(data)

            pos = part['end'][:]
            pos.append(partsColor[i])
            for j,data in enumerate(pos):
                end[j].append(data)
            
        if plt3d: 
            self.ax.scatter3D(now[0], now[1], now[2], c=now[3], marker ='s', s=44)
            self.ax.scatter3D(end[0], end[1], end[2], c=end[3], marker ='2', s=44)
        else:
            self.ax.scatter(  now[0], now[1],         c=now[3], marker ='s', s=35)
            self.ax.scatter(  end[0], end[1],         c=end[3], marker ='2', s=35)

        # Grid
        x,y,z = [],[],[]
        for xi,yi,zi in self.robot.location:
            x.append(xi)
            y.append(yi)
            z.append(zi)
        if plt3d: self.ax.scatter3D(x, y, z, marker ='.', s=8)
        else:     self.ax.scatter  (x, y,    marker ='.', s=8)

        plt.draw()
        plt.pause(0.01)


    # Inner classes (just to keep openAI gym interface)
    class ActionSpace:
        
        def __init__(self, data):
            ''' Create action space '''
            self.data = data
            
        def sample(self):
            return random.randint(0, self.data - 1)
    
    
    class ObservationSpace:
        
        def __init__(self, data):
            ''' Create observation space '''
            self.data = data
            
        def sample(self):
            return random.randint(0, self.data - 1)


if __name__ == "__main__":

    # Robot
    #robot = Robot_2A2L()
    robot = Robot_YuMi()

    # Pieces configuration
    # ... for YuMi
    piecesCfg = [{'start' : [-350, 450, 0],  # Piece 1
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
    #piecesCfg = [{'start' : [-0.5, -0.5, 0],  # Piece 1
    #              'end'   : [ 0.5, -1.5, 0],
    #             },
    #             {'start' : [-0.3, -0.8, 0],  # Piece 2
    #              'end'   : [ 1,   -1,   0],
    #             },
    #             {'start' : [-0.3, -1,   0],  # Piece 3
    #              'end'   : [ 0.3, -1,   0],
    #             },
    #             {'start' : [-0.6, -1.3, 0],  # Piece 4
    #              'end'   : [ 0.6, -0.8, 0],
    #             }]
                 #},
                 #{'start' : [-0.7, -0.7, 0],  # Piece 5
                 # 'end'   : [ 1.2, -0.4, 0],
                 #},
                 #{'start' : [-0.8, -1,   0],  # Piece 6
                 # 'end'   : [ 0.5, -1.2,   0],
                 #},
                 #{'start' : [-0.9, -0.5, 0],  # Piece 7
                 # 'end'   : [ 1.2, -0.8, 0],
                 #}]


    env = env_pickplace(robot, piecesCfg)
    env.reset()

    #
    # DEBUG section....just interesting info
    #

    # Debug - Check the number of invalid states related to pieces status
    # 317500
    #cnt=0
    #perc=0
    #print(env.nS)
    #for s in range(env.nS):
    #    env.reset(s)
    #    valid_state = env._isPiecesStatusValid()
    #    if valid_state:
    #        cnt += 1
    #print("Invalid states (due to pieces): {} ({}%)".format(cnt, 100*cnt/env.nS))

    # Debug - Check the number of invalid states related to invalid moves
    # 8040000
    #cnt=0
    #perc=0
    #offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
    #for s in range(env.nS):
    #    for a in range(env.nA):
    #        env.reset(s)
    #        joint_a = env._ext2intAction(a)
    #        for i,(a1, pos) in enumerate(zip(joint_a, env.armsGridPos)):
    #            if a1 < 4:
    #                pos = list(pos + offset_a[a1])
    #                if not env._isArmsGridPosValid(pos):
    #                    cnt +=1
    #                    break
    #print("Invalid states (boundary): {} ({}%)".format(cnt, 100*cnt/(env.nS*env.nA)))

    # Check all pick/drop invalid actions. Assume it is always invalid
    #cnt=0
    #for i in range(env.single_nA):
    #    for j in range(env.single_nA):
    #        if i==4 or i==5 or j==4 or j==5:
    #            cnt += 1
    #print("Invalid pick/drop actions: {} ({}%)".format(cnt, 100*cnt/env.nA))

    # Check all invalid actions from valid states
    # 4332504 (64%)
    #cnt = 0
    #cnt2 = 0
    #for s in range(env.nS):
    #    if env.MDP[2][s][0] == 0: # valid state
    #        cnt2 += env.nA
    #        for a in range(env.nA):
    #            if env.MDP[1][s][a] < -5:
    #                cnt += 1
    #print("Invalid actions in valid states: {} out of {} ({}%)".format(cnt, cnt2, 100*cnt/cnt2))

    # Check all valid states
    # 139520 (14%)
    #cnt = sum([1 for data in (env.MDP[2]==0) if data[0]==True])
    #print("Total valid states (from env.MDP): {} ({}%)".format(cnt, 100*cnt/len(env.MDP[2])))

    if False:
        pass
    else:
        for i in range(1000):
            if i%1 == 0:
                env.render()
            action = env.action_space.sample()
            observation,reward,done,info = env._step(action) # take a random action
            #print(i, action, info)
            print(env._ext2intState(observation), reward, done, info, action)
            if done:
                print('Done! ', i)
                break
            plt.pause(0.001)
    env.close()
