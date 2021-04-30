import numpy as np
import time    # sleep
import imageio # mimsave
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver

def showSolution(policy, initial_pos, GIF_filename=None):
    ''' Show pick & place solution '''
    init_state = robot_mdp._int2extState(initial_pos, [0,0], 15)
    next_state = init_state
    robot_mdp.reset(next_state)
    images = []
    i = 0
    done = False
    while done == False:
        # Show pick&place solution
        robot_mdp.render()
        time.sleep(0.1)
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp._step(action)

        # Show plan actions
        print(i, robot_mdp._ext2intAction(action))
        print(i, robot_mdp._ext2intState(next_state))
        i += 1

        # Create GIF
        if not (GIF_filename == None):
            image = np.frombuffer(robot_mdp.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(robot_mdp.fig.canvas.get_width_height()[::-1] + (3,)) # Example: (640,480,3)
            images.append(image)
            if done == True:
                for i in range(6):
                    images.append(images[-1]) # Just to freeze the image for a while
                imageio.mimsave(GIF_filename, images, duration=0.5)
                

def generatePythonPlan(policy, initial_pos, pieces):

    src  = '# Pieces location\n'
    src += 'pieces_location = {}\n\n'.format(pieces)

    idxs        = [(x + y*robot.M) for x,y in initial_pos]
    arms_config = [list(robot.config[i, idx]) for i,idx in enumerate(idxs)]
    arms_loc    = [list(robot.location[idx]) for idx in idxs]
    src += '# Robot init configuration\n'
    src += 'robot_init_cfg = {} # {} -> {}\n\n'.format(arms_config, initial_pos, arms_loc)

    src += 'robot_plan = [\n'
    src += '#   Configuration                                                                                                                               Actions  -  Grid location  -  Location\n'

    next_state = robot_mdp._int2extState(initial_pos, [0,0], 15)
    robot_mdp.reset(next_state)
    done = False
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp._step(action)

        # Register plan
        a_name         = ['Left ', 'Right', 'Down ', 'Up   ', 'Pick ', 'Drop ', 'Stay ']
        arms_action    = robot_mdp._ext2intAction(action)
        a_names        = [a_name[a] for a in arms_action]
        arms_pos, _, _ = robot_mdp._ext2intState(next_state)
        idxs           = [(x + y*robot.M) for x,y in arms_pos]
        arms_config    = [list(robot.config[i, idx]) for i,idx in enumerate(idxs)]
        arms_loc       = [list(robot.location[idx]) for idx in idxs]

        src += '    {},    \t# {},  {} -> {}\n'.format(arms_config, a_names, arms_pos, arms_loc)

    src += ']\n' # End of robot plan
    print(src)


def generateTxtPlan(policy, initial_pos, pieces):

    src  = '\n'
    init_state = robot_mdp._int2extState(initial_pos, [0,0], 15)
    next_state = init_state
    robot_mdp.reset(next_state)
    done = False
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp._step(action)

        # Register plan
        arms_pos, _, _ = robot_mdp._ext2intState(next_state)
        idxs           = [(x + y*robot.M) for x,y in arms_pos]
        arms_config    = [list(robot.config[i, idx]) for i,idx in enumerate(idxs)]
        arms_loc       = [list(robot.location[idx]) for idx in idxs]

        for ang in arms_config:
            print(','.join([str(x) for x in ang]))

        for pos in arms_loc:
            x,y,_ = pos
            src += '{}, {}, {}\n'.format(y,-x,250) # Axis conversion for RobotStudio

    print('\n')
    print('###############################################')
    print('# ADDITIONAL INFORMATION')
    print('#')
    print('# This plan is generated based on the next data')
    print('#')
    print('###############################################')
    print('')
    idxs           = [(x + y*robot.M) for x,y in initial_pos]
    arms_config    = [list(robot.config[i, idx]) for i,idx in enumerate(idxs)]
    arms_loc       = [list(robot.location[idx]) for idx in idxs]
    print('# Init robot location')
    print('init_grid_pos = {} # Grid location'.format(initial_pos))
    print('init_xyz_pos  = {} # XYZ location (EE)'.format(arms_loc))
    print('init_ang_pos  = {} # Robot configuration'.format(arms_config))
    print('# Copy & paste intial configuration')
    for ang in arms_config:
        print(','.join([str(x) for x in ang]))
    print('')
    print('# Pieces configuration')
    print('pieces_pos = {}'.format(pieces))

    print(" # Plan based on EE locations (instead of joints)")
    print(src)


if __name__ == "__main__":

    # YuMi
    if True:
        from robot_YuMi import Robot_YuMi
        # Robot
        robot = Robot_YuMi()

        ## Pieces configuration
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
        # Pieces configuration
        #pieces = [{'start' : [-250, 500, 0],  # Piece 1
        #           'end'   : [ 250, 200, 0],
        #          },                     
        #          {'start' : [-350, 200, 0],  # Piece 2
        #           'end'   : [ 150, 300, 0],
        #          },                     
        #          {'start' : [-50,  200, 0],  # Piece 3
        #           'end'   : [ 350, 400, 0],
        #          },                     
        #          {'start' : [-150, 400, 0],  # Piece 4
        #           'end'   : [ 50, 500, 0],
        #          }]
    # Simulated 2A2L
    #else:
    #    from robot_2A2L import Robot_2A2L
    #    robot = Robot_2A2L()
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


    # Solve MDP
    robot_mdp = mdp_generator(robot, pieces)

    if not robot_mdp.load('MDP.bin'):
        robot_mdp.generate()
        robot_mdp.save('MDP.bin')

    robot_mdp.update()

    #                      Next states       Rewards
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
    policy = solver.solve()

    # Initial pos
    init_pos   = [[1,3],[8,4]]
    init_pos   = [[3,0],[9,1]]
    
    # Show solution
    showSolution(policy, init_pos, 'robot.gif')

    # Generate helper python code for applying a solution to YuMi robot
    # DMG: need to detect step related to a piece (there is no stored arm configuration for it. Pieces location does not match arm grid locations)
    #generatePythonPlan(policy, initial_pos, pieces)
    generateTxtPlan(policy, init_pos, pieces)
