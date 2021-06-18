import numpy as np
import pickle
from robot_YuMi import Robot_YuMi


class env_hrl:
    ''' Pick&Place two-arm two-pieces '''

    def __init__(self, robot, pieces_cfg):

        #
        # Environment definition (YuMi + pieces)
        #
        self.robot     = robot
        self.piecesCfg = pieces_cfg

        #
        # This class implements an environment where there two are robots, and two pieces.
        # Robot 0 takes the piece 0 and leaves it in its target location. Robot 1 do the same with
        # the piece 1.
        # Both robots (or robot arms) move in a 2D grid (10x5).
        #
        # State space:
        # - arms position (x,y)
        # - arms status (0-searching, 1-carrying the piece, 2-done)
        #
        # S = (10*5*3) * (10*5*3) = 22500
        #
        # Action space:
        # - move (left/right/up/down), pick, drop, stay
        #
        # A = 7*7 - 1 = 48 (stay/stay makes no sense
        #
        self.M, self.N = self.robot.M, self.robot.N

        # Initial state (internal representation) 
        self.armsGridPos, self.armsStatus = self._getDefaultResetState()

        # State space size
        self.nS = (3*self.M*self.N)**2

        # Action space size: 
        self.single_nA = 7
        self.nA = self.single_nA**2 - 1 # all join actions (up/up, up/left, down/up, ...) except stay/stay
        
        # Extra internal var to indicate task DONE (so in terminal state)

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
        #self.piecesLocation = { 'start':[], 'end':[] }
        #for cfg in self.piecesCfg:
        #    # find corresponding grid location for each piece
        #    xy_i = self._nearest2DTo(cfg['start'], self.robot.location)
        #    xy_j = self._nearest2DTo(cfg['end'],   self.robot.location)

        #    self.piecesLocation['start'].append(xy_i)
        #    self.piecesLocation['end'  ].append(xy_j)

        #    # update robot grid
        #    self.robot.updateLocation(self._xy2idx(xy_i), cfg['start'])
        #    self.robot.updateLocation(self._xy2idx(xy_j), cfg['end'])

        #########
        ##
        #########


    #############################################################
    #
    # Helper functions
    #
    #############################################################

    def setPiecesLocation(self, piecesCfg):

        self.piecesLocation = { 'start':[], 'end':[] }
        for cfg in piecesCfg:
            if cfg == {}:
                xy_i = []
                xy_j = []
            else:
                # find corresponding grid location for each piece
                xy_i = self._nearest2DTo(cfg['start'], self.robot.location)
                xy_j = self._nearest2DTo(cfg['end'],   self.robot.location)

            self.piecesLocation['start'].append(xy_i)
            self.piecesLocation['end'  ].append(xy_j)
#            print(cfg)
#        print(self.piecesLocation)

    def setPiecesRobotLocation(self, piecesCfg):
        ''' Update robot locations (xyz) for each piece location (pieces probably won't
            be on an exact 2D grid location '''

        for cfg in piecesCfg:
            # find corresponding grid location for each piece
            xy_i = self._nearest2DTo(cfg['start'], self.robot.location)
            xy_j = self._nearest2DTo(cfg['end'],   self.robot.location)

            # update robot grid
            self.robot.updateLocation(self._xy2idx(xy_i), cfg['start'])
            self.robot.updateLocation(self._xy2idx(xy_j), cfg['end'])
        

    def _int2extState(self, armsPos, armsStatus):
        ''' convert internal state representation to a single scalar '''
        N,M = self.N, self.M

        state = 0

        tmp = 0
        for status in armsStatus:
            tmp *= 3
            tmp += status
        state += tmp

        tmp = 0
        for x,y in armsPos:
            tmp *= (N*M)
            tmp += self._xy2idx([x,y])
        state += tmp * 9

        return state

    def _ext2intState(self, state):
        ''' convert external state (scalar) to internal state representation '''
        N,M = self.N, self.M

        arms_status = []
        for _ in self.armsStatus:
            tmp = state % 3
            arms_status.insert(0, tmp)
            state = (state - tmp) // 3

        arms_pos = [] 
        for _ in self.armsGridPos:
            idx = state % (N*M)
            state = (state - idx) // (N*M) 
            arms_pos.insert(0, self._idx2xy(idx))

        return [arms_pos, arms_status]

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

    def _getDefaultResetState(self):
        armsGridPos = [[0,0], [6,4]]  # init 2D grid pos
        armsStatus  = [0, 0]          # init status: carrying no piece
        return [armsGridPos, armsStatus]

    def _isArmsGridPosValid(self, pos):
        ''' Check if pos is inside the grid boundaries '''
        x,y = pos
        return not ((x >= self.M) or (x < 0) or (y >= self.N) or (y < 0))

    def reset(self, state=None):
        ''' Bring environment to the default state '''

        if state == None:
            self.armsGridPos, self.armsStatus = self._getDefaultResetState()
        else:
            self.armsGridPos, self.armsStatus = self._ext2intState(state)

        #self.piecesCurrPos = [ cfg for cfg in self.piecesLocation['start'] ]

    def _isStateValid(self, state):
        # Before processing an action over the current state, let's check if the 
        # state is valid. Invalid states will not be involved in the RL game.
        #
        # Invalid states:
        # - Robot unable to reach the pos represented by the current state
        # - Robot arms collission
        #
        # Info response field will indicate this issue: 'invalid'
        #

        # Trick...this states are valid, but any move from them is invalid:w
        #if state>=3897 and state<=3904:
        #    print('Trick ' ,state)
        #    return False

        self.reset(state)

        valid_state = self.robot.checkValidLocation(self.armsGridPos)

        if valid_state:
            valid_state = not self.robot.checkCollision(self.armsGridPos)

        return valid_state

    def step(self, action):
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
            state  = self._int2extState(self.armsGridPos, self.armsStatus)
            reward = 0
            done   = True
            info   = ''
            return state, reward, done, info

        # Now, let's see what happens when applying the agent action. Wrong 
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
        info            = ''
        done            = False

        next_arms_grid_pos   = []
        #next_arms_status     = self.armsStatus[:]                 # copy 1D list
        move_both_arms       = True 

        offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up

        joint_a = self._ext2intAction(action)
        for i,(a, pos) in enumerate(zip(joint_a, self.armsGridPos)):
            # move (right/left/down/up)
            if a < 4:
                pos = list(pos + offset_a[a])
                if not self._isArmsGridPosValid(pos):
                    valid_state = False # Grid boundaries exceeded
                    break

            # pick or place
            elif a == 4 or a == 5:
                # At this point, valid pick/drop locations are unknown (so all invalid)
                # Thanks to this, we can ignore also the armsStatus update, because it only happens when these actions
                valid_state = False
            # stay
            else:
                move_both_arms  = False # One arm stays still

            next_arms_grid_pos.append(pos)

        if valid_state:
            # Check if both arms can reach their new position
            valid_state = self.robot.checkValidLocation(next_arms_grid_pos)

        if valid_state:
            # Check if both arms collide
            valid_state = not self.robot.checkCollision(next_arms_grid_pos)

        if valid_state:
            self.armsGridPos   = next_arms_grid_pos
            #self.armsStatus    = next_arms_status


            if self._isGoalMet():
                reward = 10000
                done   = True
                print('GOAL MET ', self.armsGridPos)
            else:
                # Different reward when moving one or both arms helps to:
                # - prefer moving both arms at once (-2) than separately (-3.8)
                # - prefer moving only one arm when the other already got 
                #   its goal (-1.9 better than -2)
                if move_both_arms == True: reward = -1
                else:                      reward = -1
        else:
            # The same reward (punishment) for any unwanted action
            reward = -20

        # Build response
        observation = self._int2extState(self.armsGridPos, self.armsStatus)


        return observation, reward, done, info

    def _isGoalMet(self):
        ''' Is MDP solved? '''
        return self.armsStatus == [2,2]
      
        #TODO: define a goal
        #return False
        #return (np.sum(self.armsStatus) == 0) and (self.armsGridPos[0] == self.piecesLocation['start'][0]) and (self.armsGridPos[1] == self.piecesLocation['start'][1])
        #return (np.sum(self.armsStatus) == 1) and (self.armsGridPos[0] == self.piecesLocation['end'][0]) and (self.armsGridPos[1] == self.piecesLocation['end'][1])

    def render(self):
        ''' Show environment '''
        print(self.armsGridPos, self.armsStatus)

    def close(self):
        ''' Close environment '''
        pass

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

    env = env_hrl(robot, piecesCfg)
