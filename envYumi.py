
import time
import random
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.backend_bases import MouseButton
from yumi import YuMi
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

class EnvYuMi:
    ''' YuMi Pick&Place MDP environment
    
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
    def __init__(self, pieces_cfg=None):

        #
        # Environment definition (YuMi + pieces)
        #
        self.robot = YuMi()

        if pieces_cfg == None:
            self.piecesCfg = [{'start' : [-0.5, -0.5, 0],  # Piece 1
                               'end'   : [ 0.5, -1.5, 0],
                               'nArms' : 1
                              },
                              {'start' : [-0.3, -0.8, 0],  # Piece 2
                               'end'   : [ 1,   -1,   0],
                               'nArms' : 1
                              },
                              {'start' : [-0.3, -1,   0],  # Piece 3
                               'end'   : [ 0.3, -1,   0],
                               'nArms' : 1
                              },
                              {'start' : [-0.6, -1.3, 0],  # Piece 4
                               'end'   : [ 0.6, -0.8, 0],
                               'nArms' : 1
                              }]
            #self.piecesCfg = [{'start' : [-1.1, -0.8, 0],  # Piece 1
            #                   'end'   : [ 0.5, -1.5, 0],
            #                   'nArms' : 1
            #                  },
            #                  {'start' : [-0.5, -0.2, 0],  # Piece 2
            #                   'end'   : [ 1,   -1,   0],
            #                   'nArms' : 1
            #                  },
            #                  {'start' : [-0.6, -0.8,   0],  # Piece 3
            #                   'end'   : [ 0.3, -1,   0],
            #                   'nArms' : 1
            #                  },
            #                  {'start' : [-0.3, -1, 0],  # Piece 4
            #                   'end'   : [ 0.6, -0.8, 0],
            #                   'nArms' : 1
            #                  }]
                              #},
                              #{'start' : [-0.7, -0.7, 0],  # Piece 5
                              # 'end'   : [ 1.2, -0.4, 0],
                              # 'nArms' : 1
                              #},
                              #{'start' : [-0.8, -1,   0],  # Piece 6
                              # 'end'   : [ 0.5, -1.2,   0],
                              # 'nArms' : 1
                              #},
                              #{'start' : [-0.9, -0.5, 0],  # Piece 7
                              # 'end'   : [ 1.2, -0.8, 0],
                              # 'nArms' : 1
                              #}]
        else:
            self.piecesCfg = pieces_cfg


        # Robot for testing moves without affecting environment robot
        self.test_robot = YuMi()

        # MDP
        #
        # YuMi robot is a two arm robot with 14 free dregree (seven per arm).
        # 
        # This MDP, by nature, should be modeled with continuous action space 
        # (action ending in any arbitrary 14-angles configuration) and 
        # continuous state space (any 14-angles configuration).
        #
        # Simplifications:
        # - Do not work with robot configuration (angles) but with its EE 
        #   position. This will require to precompute robot configuration 
        #   (angles) for all EE potential positions.
        #   This way, no matter the number of degrees of freedom of the robot,
        #   but the EE positions
        # - Discretize state space: N*M grid (EndEffector 2D positions).
        #   Pick&Place operations are considered atomic operations (ignored the
        #   up/down movement in the state space)
        # - Discretize action space: up/down/left/rigth/pick/place/stay 
        #   (movements constrained to reach adjacent grid cells)
        # - Assume pick and place operations last the same as any other move
        # - Do not base MDP reward on arms movement interval time. Simplify
        #   considering '-1' reward per each move.
        #
        # Note: This will generate a very discretized trayectory. It will 
        #       require posterior processing to smooth it.
         
         
        # State space:
        # - Arms move in a 2D grid N*M (init and target pieces positions will 
        #   replace some of the standard grid positions)
        #
        #      ____ piece 1 init pos
        #     /
        #   . v .   .   .
        #   . .     .   .
        #   .   .   .   .
        #
        # - MDP state is represented by a scalar number built from internal state representation:
        #   - arms position (x,y,z)
        #   - arms status (carrying piece i or none)
        #   - pieces status (pieces pending to be picke up)
        #
        # Note: part of the code is specific for two-arm robots (TODO: refactor)
        #
        N,M = 10,5 # (cols, rows)
        #N,M = 21,10 # (cols, rows)
        K = len(self.piecesCfg)
        self.N, self.M, self.K = N,M,K
 
        self.armsGridPos, self.armsStatus, self.piecesMap = self._getDefaultResetState()

        self.nS = ((N*M)**2) * (2**K) * ((K+1)**2)

        # Action space:
        # - Any combination of both arm actions (up/up, up/left, down/up, ... except stay/stay)
        self.single_nA = 7
        self.nA = self.single_nA**2 - 1

        # Precompute robot (angles) configuration for all arms positions
        #
        # 1. Define 2D grid of EE positions
        #    - data manually selected from YuMi robot data
        grid_x, grid_y =  -1.35, -0.2  # top-left 2D grid corner
        step_x, step_y =   0.3,  -0.3  # distance between cells
        #grid_x, grid_y =  -1.5,  -0.2   # top-left 2D grid corner
        #step_x, step_y =   0.15, -0.15  # distance between cells

        self.grid = [ [grid_x + step_x*j, grid_y + step_y*k, 0] for j in range(N) for k in range(M)]

        # 2. The pieces will be part of the grid, keeping the N*M logical grid structure.
        #    However, physically, grid cells will not be equidistant anymore.
        #
        #             .   .   .   .         .   .   .   .
        #             .   .   .   .   -->   . .     .   .
        #             .   .   .   .         .   .   .   .
        #
        # Note: Risk! several pieces may fall in the same cell
        #
        self.piecesGridCfg = []
        for cfg in self.piecesCfg:
            # piece i initial pos
            idx_i = self._nearestTo(cfg['start'], self.grid)
            self.grid[idx_i] = cfg['start']

            # piece i final pos
            idx_j = self._nearestTo(cfg['end'], self.grid)
            self.grid[idx_j] = cfg['end']

            # build piecesGridCfg (grid cells equivalence)
            pgCfg = {
                      'start': self._idx2xy(idx_i), 
                      'end'  : self._idx2xy(idx_j)
                    }
            self.piecesGridCfg.append(pgCfg)

        # 3. Compute corresponding robot (angles) configurations
        self.robot.setAngles([[np.pi, 0.3], [0, -1]]) # Initial position (very near to first position)
        
        nArms = self.robot.getNumArms()
        self.gridAng = [ [] for _ in range(nArms)]
        for pos in self.grid:
            self.robot.setEE([pos for _ in range(nArms)])
            angles = self.robot.getAngles()
            for i,ang in enumerate(angles):
                self.gridAng[i].append(ang)

        # Pieces initial pos
        self.piecesGridPos = [ cfg['start']  for cfg in self.piecesGridCfg ]

        # Interactive mode
        self.selectedArm = 0

        # Open AI gym-style stuff)
        self.action_space      = self.ActionSpace(self.nA)
        self.observation_space = self.ObservationSpace(self.nS)
        
        # Init environment vars
        self._initRuntimeVars()

        # Speed-up helpers
        self.reachable = [{} for _ in range(nArms)]
        self.collision = {}
        self.piecesStatusValid = {}

        # Build or load previously build Datamodel
        self._buildOrLoadMDP()

    def _buildMDP(self):

        print("Filtering valid states ...")

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
        for i,s in enumerate(valid_states):
            for a in range(self.nA):
               self.reset(s)
               next_state, reward, done, info = self._step(a)
               mdp_s[i][a] = states_idx[next_state]
               mdp_r[i][a] = reward
            if (i % (valid_nS//10)) == 0:
                print(10*i//(valid_nS//10),'%')

        mdp = [mdp_s, mdp_r, mdp_v]

        return mdp

    def _buildOrLoadMDP(self):
        ''' Build and store to file the MDP model for the first time.
            Load the MDP model from file the next time
        '''
        filename = 'MDP.bin'
        try:
            # Load stored MDP model
            with open(filename, 'rb') as f:
                self.MDP = pickle.load(f)
        except:
            # Build MDP model
            self.MDP = self._buildMDP()

            # Store MDP model
            with open(filename, 'wb') as f:
                pickle.dump(self.MDP, f)


    def _initRuntimeVars(self):

        # Robot initial pose (no load)
        self.robot.reset()


    def _int2extState(self, armsPos, armsStatus, piecesMap):
        ''' convert internal state representation to a single scalar '''
        N,M,K = self.N, self.M, self.K

        state = piecesMap

        tmp = 0
        for status in armsStatus:
            tmp *= (K + 1)
            tmp += status
        state += tmp * (2**K)

        tmp = 0
        for x,y in armsPos:
            tmp *= (N*M)
            tmp += self._xy2idx([x,y])
        state += tmp * (2**K) * ((K+1)**2)

        return state

    def _ext2intState(self, state):
        ''' convert external state (scalar) to internal state representation '''
        N,M,K = self.N, self.M, self.K

        pieces_map = state % (2**K) 
        state = (state - pieces_map) // (2**K)

        arms_status = []
        for _ in self.armsStatus:
            tmp = state % (K+1)
            arms_status.insert(0, tmp)
            state = (state - tmp) // (K+1)

        arms_pos = [] 
        for _ in self.armsGridPos:
            idx = state % (N*M)
            state = (state - idx) // (N*M) 
            arms_pos.insert(0, self._idx2xy(idx))

        return [arms_pos, arms_status, pieces_map]

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

    # Auxiliary functions
    #
    def _nearestTo(self, item, item_list):
        ''' Find the most similar entry in the item_list '''
        dist = [ np.sqrt(sum( (a - b)**2 for a, b in zip(item, p))) for p in item_list]
        return np.argmin(dist) 

    def _xy2idx(self, xy):
        ''' xy grid pos to index '''
        x,y = xy
        return x*self.M + y

    def _idx2xy(self, idx):
        ''' xy grid pos to index '''
        return [idx // self.M, idx % self.M]

    def _logic2phy(self, pos):
        ''' convert logical grid cell pos to physical 3D point
            [0,1] -> [-1.35, -0.5, 0]
        '''
        return self.grid[ self._xy2idx(pos) ]

    def _phy2logic(self, pos):
        ''' convert physical 3D point to logical grid cell pos
            [-1.05, -0.8, 0] -> [1,2]
        '''
        # Do not compare floats. Find the most similar cell pos
        idx = self._nearestTo(pos, self.grid)
        return self._idx2toxy(idx)

    def _logic2ang(self, arm_idx, pos):
        ''' convert logical grid cell pos to robot angles configuration
            [0,1] -> [2.61814353, 2.11034636]
        '''
        return self.gridAng[arm_idx][ self._xy2idx(pos) ]

    def _isArmsGridPosValid(self, pos):
        ''' Check if pos is inside the grid boundaries '''
        x,y = pos
        N,M = self.N, self.M
        if ((x >= N) or (x < 0) or (y >= M) or (y < 0)):
            return False
        else:
            return True


    def _isRobotPosReachable(self, armsGridPos):
        ''' Check if Arms EEs can reach their target pos '''

        # Memoization technique (save and reuse previous computations)


        # Check physical EE locations
        res = True
        #phyPos = [self.grid[ self._xy2idx(pos) ] for pos in armsGridPos]
        #for pos, cfg in zip(phyPos, self.robot.cfg):
        for i, (lpos, cfg) in enumerate(zip(armsGridPos, self.robot.cfg)):
            idx = self._xy2idx(lpos)
            phyPos = self.grid[idx]
            # Memoization (reuse previous checks))
            if idx in self.reachable[i].keys():
                if self.reachable[i][idx] == False:
                    res = False
                    break
            # Check it (and store check)
            else:
                if np.linalg.norm(np.array(phyPos) - np.array(cfg['modelPos'])) >= 1.95: # two links 1m each (particular to default config)
                    res = False
                    self.reachable[i][idx] = False
                    break
                self.reachable[i][idx] = True

        return res

    def _isPiecesStatusValid(self, armsGridPos, armsStatus, piecesMap):
        ''' Check pieces status coherence '''

        # Memoization
        #idx = self._int2extState([[0,0],[0,0]], self.armsStatus, self.piecesMap) # luckily we can reuse this function (setting higher component to 0s)
        #if idx in self.piecesStatusValid.keys():
        #    print('j', idx)
        #    return self.piecesStatusValid[idx]

        # Perform the checks
        res = True

        # - incoherence: the same piece carried by several arms
        picked = [x for x in armsStatus if x>0]
        if len(picked) != len(list(set(picked))):
            res = False

        # - incoherence: a piece has not been picked up yet, and it is carried by an arm at the same time
        if res == True:
            for piece in armsStatus:
                if (piece > 0) and (piecesMap & (1<<(piece-1))):
                    res = False
                    break

        # - When the job is done, at least one arm should be at a target piece position
        if res == True:
            if (piecesMap == 0) and np.sum(armsStatus) == 0:
                tmp = [ p1==p2['end'] for p1 in armsGridPos for p2 in self.piecesGridCfg ]
                res = True in tmp


        # and store it
        #self.piecesStatusValid[idx] = res

        return res

    def _isPickUpActionValid(self, arm_grid_pos, arm_status, pieces_map):
        ''' Check pick up action coherence in the current state for a specific arm '''
        valid = False
        piece = 0

        if arm_status == 0:
            for i,piece_pos in enumerate(self.piecesGridCfg):
                if pieces_map & 0x01:
                    if piece_pos['start'] == arm_grid_pos:
                        valid = True
                        piece = i
                        break
                pieces_map >>= 1

        return valid, piece

    def _isDropOffActionValid(self, arm_grid_pos, arm_status):
        ''' Check drop off action coherence in the current state for a specific arm '''
        valid = False

        if arm_status > 0:
            piece_pos = self.piecesGridCfg[arm_status-1]        
            if piece_pos['end'] == arm_grid_pos:
                valid = True

        return valid

    def _isThereCollision(self, armsGridPos):
        # Memoization technique (save and reuse previous computations)
        N,M,K = self.N, self.M, self.K
        collision_idx = 0
        for x,y in armsGridPos:
            collision_idx *= (N*M)
            collision_idx += self._xy2idx([x,y])

        if collision_idx in self.collision.keys():
            return self.collision[collision_idx]

        # Compute collision
        robot_ang = [grid_ang[self._xy2idx(grid_pos)] for grid_pos, grid_ang in zip(armsGridPos, self.gridAng)]
        collision = self.robot._checkCollision(robot_ang, 0.40)

        # Store computation
        self.collision[collision_idx] = collision

        return collision

    def _isGoalMet(self):
        ''' Is MDP solved? '''
        return ((self.piecesMap == 0) and (np.sum(self.armsStatus) == 0))
        #if self.piecesMap != 0:
        #    return False
        #for status in self.armsStatus:
        #    if status > 0:
        #        return False
        #
        ## All pieces picked up and dropped off
        #return True

    def _getDefaultResetState(self):
        armsGridPos = [[0,0], [6,4]]  # init 2D grid pos
        armsStatus  = [0, 0]          # init status: carrying no piece
        piecesMap   = (2**self.K) - 1 # init status: No piece has been picked up yet
        return [armsGridPos, armsStatus, piecesMap]

    def reset(self, state=None):
        ''' Bring environment to the default state '''

        if state == None:
            self.armsGridPos, self.armsStatus, self.piecesMap = self._getDefaultResetState()
        else:
            self.armsGridPos, self.armsStatus, self.piecesMap = self._ext2intState(state)

    def render(self):
        ''' Show environment '''
        # Update robot configuration
        pos = [self._logic2ang(i, grid_pos) for i,grid_pos in enumerate(self.armsGridPos)]
        self.robot.setAngles(pos)

        # Plot robot
        self._plot()

    def step(self, action):
        ''' Take an action. Make partial processing. Use reward data from 
            self.MDP to avoid the heavy processing part. This code is a reduced
            version of '_step()' function.

            The point is to generate and store a MDP containing only reward, 
            done and invalid data (all in the same byte) to reduce memory
            footprint. Then, next_state data is computed on the fly in each 
            function call.
        '''
        done   = False
        info   = ''

        state_idx = {}
        for i, item in enumerate(self.MDP[2]):
            state_idx[item] = i

        state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap)
        reward = self.MDP[1][state_idx[state]][action]

        # Goal completed. Do nothing (reward is 0)
        #
        if self._isGoalMet():
            reward = 0
            done   = True

        # Invalid next_state
        elif reward <= -5: # TODO: better to define this as a macro
            pass

        # Valid next_state
        else:
            offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up
            joint_a = self._ext2intAction(action)

            for i,(a, pos) in enumerate(zip(joint_a, self.armsGridPos)):
                # move right/left/down/up
                if a < 4: 
                    pos = list(pos + offset_a[a])
                    self.armsGridPos[i] = pos
                    piece_idx = self.armsStatus[i]
                    if piece_idx > 0:
                        self.piecesGridPos[piece_idx-1] = pos
                # pick up
                elif a == 4:
                    _, piece_idx = self._isPickUpActionValid(pos, self.armsStatus[i], self.piecesMap)
                    self.armsStatus[i]  = piece_idx + 1
                    self.piecesMap     &= ~(1<<piece_idx)
                # drop off
                elif a == 5:
                    self.armsStatus[i]  = 0

            state = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap)

        return state, reward, done, info

    def _isStateValid(self, state):
        # Before processing an action over the current state, let's check if the 
        # state is valid. Invalid states will not be involved in the RL game.
        #
        # Invalid states:
        # - Robot unable to reach the pos represented by the current state
        # - Robot arms collission
        # - Invalid piece status (internal state representation issue)
        #
        # Info response field will indicate this issue: 'invalid'
        #
        self.reset(state)

        valid_state = self._isRobotPosReachable(self.armsGridPos)

        if valid_state:
            valid_state = not self._isThereCollision(self.armsGridPos)

        if valid_state:
            valid_state = self._isPiecesStatusValid(self.armsGridPos, self.armsStatus, self.piecesMap)

        return valid_state
        #if valid_state == False:
        #    # Abort wih error
        #    state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap) # irrelevant
        #    reward = 0     # irrelevant
        #    done   = False # irrelevant
        #    info   = 'invalid'
        #    return state, reward, done, info

    def _step(self, action, mode=0):
        ''' Take an action. Make the full processing

            action: scalar in the range [0, self.nA-1]
            mode  : 0 (normal), 1 (pick&place always succeeds), 2 (pick and place always fails)

            PickPlace trick must be done per piece
        '''

        # Current state is valid. Let's process the action!
        
        # First, check if we've already met the goal
        #
        if self._isGoalMet():
            # Do not process the action. By design, any action results in the
            # same goal state with reward 0.
            state  = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap)
            reward = 0
            done   = True
            info   = ''
            return state, reward, done, info

        # Now, let's see what happens when applying the agent action. Wrong 
        # actions will be punished, while good ones will be # reinforced
        # (reward reponse field)
        #
        # Wrong action reasons:
        # - Robot exceeds grid boundaries
        # - Incoherent pick up or drop off action (nothing to pick up or drop off)
        # - Robot arms collision
        #
        next_arms_status = self.armsStatus[:] # copy 1D list
        next_pieces_map  = self.piecesMap
        next_pieces_grid_pos = [x[:] for x in self.piecesGridPos] # copy 2D list
        move_both_arms   = True 
        info        = ''
        done        = False
        valid_state = True
        pick_place_mark = False

        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up

        joint_a = self._ext2intAction(action)
        next_pos = []
        for i,(a, pos) in enumerate(zip(joint_a, self.armsGridPos)):
            # move right/left/down/up
            if a < 4: 
                pos = list(pos + offset_a[a])
                if not self._isArmsGridPosValid(pos):
                    valid_state = False # Grid boundaries exceeded
                    break
                # update piece pos
                piece_idx = self.armsStatus[i]
                if piece_idx > 0:
                    next_pieces_grid_pos[piece_idx-1] = pos

            # Do NOT analyze pick and place actions (in any state). Consider them as invalid, 
            # Specifically, if any check fails, do not make anything special (reward = -20),
            # but if every check is passed, then return a reward=-30. This will help to 
            # identify which state/pair actions can be corrected.
            #
            # The idea behind this is to precompute as much as possible the MDP table, 
            # and then, when pieces configuration is known (real time), complete it
            # (updating those 'marked' state/pair actions corresponding to the pieces
            # locations).
            # This trick will speed up the algorithm in real time (most of the MDP table
            # will be precocomputed offline)
            #
            # pick or place
            #
            elif a == 4 or a == 5:
                # mark it, but do not abort the remaining checks
                pick_place_mark = True
            
            # As mentioned above, do not analyze these actions for a specific sceneario
            # (pieces configuration)
            #
            ## pick up
            #elif a == 4:
            #    valid_state, piece_idx = self._isPickUpActionValid(pos, self.armsStatus[i], self.piecesMap)
            #    if valid_state:
            #        next_arms_status[i]  = piece_idx + 1
            #        next_pieces_map     &= ~(1<<piece_idx)
            #    else:
            #        break # Nothing to pick up
            ## drop off
            #elif a == 5:
            #    valid_state = self._isDropOffActionValid(pos, self.armsStatus[i])
            #    if valid_state:
            #        next_arms_status[i]  = 0
            #    else:
            #        break # Nothing to drop off
            ## stay
            else:
                move_both_arms  = False # One arm stays still

            next_pos.append(pos)

        if valid_state:
            valid_state = self._isRobotPosReachable(next_pos)

        if valid_state:
            valid_state = not self._isThereCollision(next_pos)

        if valid_state:
            valid_state = self._isPiecesStatusValid(next_pos, next_arms_status, next_pieces_map)

        if valid_state:
            if pick_place_mark:
                # special reward mark (state unchanged)
                reward = -30
            else:
                self.armsStatus    = next_arms_status
                self.piecesMap     = next_pieces_map
                self.armsGridPos   = next_pos
                self.piecesGridPos = next_pieces_grid_pos

                if self._isGoalMet():
                    reward = 100
                    done   = True
                else:
                    # Different reward when moving one or both arms helps to:
                    # - prefer moving both arms at once (-2) than separately (-3.8)
                    # - prefer moving only one arm when the other already got 
                    #   its goal (-1.9 better than -2)
                    if move_both_arms == True: reward = -3
                    else:                      reward = -2
        else:
            # The same reward (punishment) for any unwanted action
            reward = -20

        # Build response
        observation = self._int2extState(self.armsGridPos, self.armsStatus, self.piecesMap)


        return observation, reward, done, info

    def close(self):
        ''' Close environment '''
        pass

    def setManualMode(self):
        ''' Enter interactive mode.
            Use the mouse to interact with the environment:
            - Left mouse click : move selected arm to mouse pos
            - Right mouse click: select arm (nearest to mouse pos)
        '''
        self.done = False
        self._plot()

        cid = self.fig.canvas.mpl_connect('button_press_event', self._manualModeOnclick)

    def _manualModeOnclick(self, event):

        if self.done == True:
            self.done = False
            self.reset()
        else:
            mouse = np.array([event.xdata, event.ydata, 0])

            # Right click to select arm (the nearest one)
            if event.button == MouseButton.RIGHT:
                self.selectedArm = self._nearestTo(mouse, self.robot.getEE())

            # Left click to move the arm
            else:
                # Get the grid cell nearest to the mouse selection
                idx = self._nearestTo(mouse, self.grid)

                # Update robot configuration
                ang = self.robot.getAngles()
                ang[self.selectedArm] = self.gridAng[self.selectedArm][idx]
                self.robot.setAngles(ang)


        self._plot()

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
                self.fig = plt.gcf()

        plt.axis([-3,3,-3,3]) # TODO: no poner a fuego

        # Show the environment
        #robotColor = 'r' if self.robot._checkCollision(0.3) else 'grey'
        robotColor = 'grey'

        # Robot
        for vertices in self.robot.getLinksPos():
            x,y,z =  np.array(vertices).T

            if plt3d:
                self.line, = self.ax.plot3D(x, y, z, c=robotColor, lw=8)
                self.line, = self.ax.plot3D(x, y, z, c='r', marker='o')
            else:
                self.line, = self.ax.plot(x, y, c=robotColor, lw=8)
                self.line, = self.ax.plot(x, y, c='r', marker='o')

        # Pieces
        partsColor ='bgrcmykbgrcmyk'
        now, end = [[],[],[],[]], [[],[],[],[]] # x,y,z,color
        for i, (grid_pos, part) in enumerate(zip(self.piecesGridPos, self.piecesCfg)):
            pos = self._logic2phy(grid_pos)[:]
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
        for vertices in self.grid:
            xi,yi,zi =  np.array(vertices).T
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
    # Example (redefine environment: 4 pieces)
    #pieces_cfg = [{'start' : [-0.5, -0.5, 0],  # Piece 1
    #               'end'   : [ 0.5, -1.5, 0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.3, -0.8, 0],  # Piece 2
    #               'end'   : [ 1,   -1,   0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.3, -1,   0],  # Piece 3
    #               'end'   : [ 0.3, -1,   0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.6, -1.3, 0],  # Piece 4
    #               'end'   : [ 0.6, -0.8, 0],
    #               'nArms' : 1
    #              }]
    #              #},
    #              #{'start' : [-0.7, -0.7, 0],  # Piece 5
    #              # 'end'   : [ 1.2, -0.4, 0],
    #              # 'nArms' : 1
    #              #},
    #              #{'start' : [-0.8, -1,   0],  # Piece 6
    #              # 'end'   : [ 0.5, -1.2,   0],
    #              # 'nArms' : 1
    #              #},
    #              #{'start' : [-0.9, -0.5, 0],  # Piece 7
    #              # 'end'   : [ 1.2, -0.8, 0],
    #              # 'nArms' : 1
    #              #}]
    #env = EnvYuMi(pieces_cfg)
    env = EnvYuMi()
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

    if True:
        env.setManualMode()
    else:
        for i in range(2000):
            if i%20001 == 0:
                env.render()
            action = env.action_space.sample()
            _,_,done,info = env.step(action) # take a random action
            #print(i, action, info)
            if done:
                print('Done! ', i)
                break
            plt.pause(0.001)
    env.close()
