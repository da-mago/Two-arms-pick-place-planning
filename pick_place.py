import os
import numpy as np
import time    # sleep
#import imageio # mimsave
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver

def mdp_step(state_rd, action):
    '''debug function'''
    next_state_rd = robot_mdp.MDP[0][state_rd][action]
    next_state = robot_mdp.MDP[2][next_state_rd]
    reward     = robot_mdp.MDP[1][state_rd][action]
    done       = True if reward>100 else False
    info       = ''
    return next_state, reward, done, info

def showSolution(policy, initial_pos, GIF_filename=None):
    ''' Show pick & place solution '''
    global num_steps

    init_state = robot_mdp._int2extState(initial_pos, [0,0], 15, [0,0])
    next_state = init_state
    robot_mdp.reset(next_state)
    images = []
    i = 0
    done = False
    print('SOLUTION', initial_pos)
    while done == False:

        num_steps += 1
        # Show pick&place solution
        #robot_mdp.render()
        #time.sleep(0.1)
        state_rd   = robot_mdp.MDP[3][next_state]
        action = policy[state_rd]
        next_state, reward, done, info = robot_mdp._step(action)
        #next_state, reward, done, info = mdp_step(state_rd, action)
       
        #next_state_rd = robot_mdp.MDP[0][state_rd][action]
        #next_state = robot_mdp.MDP[2][next_state_rd]
        #reward     = robot_mdp.MDP[1][state_rd][action]
        #done       = True if reward>100 else False
        #info       = ''

        # Show plan actions
        #print(i, robot_mdp._ext2intAction(action))
        #print(i, robot_mdp._ext2intState(next_state))
        pos,status,pieces_map,pick_pos = robot_mdp._ext2intState(next_state)
        joint_a = robot_mdp._ext2intAction(action)
        print(joint_a, pos, ',', status, ',', pieces_map, ',', pick_pos, i)
        for arm_idx in range(1,-1,-1):
            idx = robot_mdp._xy2idx(pos[arm_idx])
            x,y,_       = robot_mdp.robot.location[idx]
            #arm_config = robot.config[arm_idx, 0,  idx]
            #print("{}, {}, 250".format(y, -x))
        i += 1

        # Forced exit (no solution)
        if i>100:
            return

        # Create GIF
        if not (GIF_filename == None):
            image = np.frombuffer(robot_mdp.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(robot_mdp.fig.canvas.get_width_height()[::-1] + (3,)) # Example: (640,480,3)
            images.append(image)
            if done == True:
                for _ in range(6):
                    images.append(images[-1]) # Just to freeze the image for a while
                imageio.mimsave(GIF_filename, images, duration=0.5)

    #print(f"Solution in {i}")
                

def generatePythonPlan(policy, initial_pos, pieces):

    src  = '# Pieces location\n'
    src += 'pieces_location = {}\n\n'.format(pieces)

    idxs        = [(x + y*robot.M) for x,y in initial_pos]
    arms_config = [list(robot.config[i, 0, idx]) for i,idx in enumerate(idxs)]
    arms_loc    = [list(robot.location[idx]) for idx in idxs]
    src += '# Robot init configuration\n'
    src += 'robot_init_cfg = {} # {} -> {}\n\n'.format(arms_config, initial_pos, arms_loc)

    src += 'robot_plan = [\n'
    src += '#   Configuration                                                                                                                               Actions  -  Grid location  -  Location\n'

    next_state = robot_mdp._int2extState(initial_pos, [0,0], 15, [0,0])
    robot_mdp.reset(next_state)
    done = False
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp._step(action)

        # Register plan
        a_name         = ['Left ', 'Right', 'Down ', 'Up   ', 'Pick ', 'Drop ', 'Stay ']
        arms_action    = robot_mdp._ext2intAction(action)
        a_names        = [a_name[a] for a in arms_action]
        arms_pos, _, _, _ = robot_mdp._ext2intState(next_state)
        idxs           = [(x + y*robot.M) for x,y in arms_pos]
        # TODO: this function does not take care of different Zs grids
        arms_config    = [list(robot.config[i, 0, idx]) for i,idx in enumerate(idxs)]
        arms_loc       = [list(robot.location[idx]) for idx in idxs]

        src += '    {},    \t# {},  {} -> {}\n'.format(arms_config, a_names, arms_pos, arms_loc)

    src += ']\n' # End of robot plan
    print(src)


def generateTxtPlan(policy, initial_pos, pieces):

    # Sort of MACROs
    GRIPPER_XY    = 0
    GRIPPER_DOWN  = 1
    GRIPPER_CLOSE = 2
    GRIPPER_OPEN  = 3
    GRIPPER_UP    = 4
    Z_PLANE       = 180
    Z_UPDOWN      = 110
    Z_GRIPPER     = Z_UPDOWN

    src  = '\n'
    # Initial position
    #for (pos_x, pos_y) in reversed(initial_pos):
    #    z = Z_PLANE
    #    idx = pos_x + pos_y*robot_mdp.M
    #    x,y,_ = robot.location[idx]
    #    src += '{} {} {}\n'.format(y,-x,z) # Axis conversion for RobotStudio

    num_steps = 0
    bitmap_init = (2**len(pieces)) - 1
    init_state = robot_mdp._int2extState(initial_pos, [0,0], bitmap_init, [0,0])
    next_state = init_state
    robot_mdp.reset(next_state)

    print("Generating plan ...")
    idxs = [(x + y*robot_mdp.M) for (x,y) in initial_pos]
    arms_config = [list(robot.config[i, 0, idx]) for i,idx in enumerate(idxs)]
    #for ang in reversed(arms_config):
    #    print(','.join([str(x) for x in ang]))

    for ang,(pos_x, pos_y) in zip(reversed(arms_config), reversed(initial_pos)):
        # Format:
        #   ANGLES # GRIPPER (gripper open/close) or XYZ (MOVE Z)
        z = Z_PLANE
        src += ','.join([str(x) for x in ang]) 
        idx = pos_x + pos_y*robot_mdp.M
        x,y,_ = robot.location[idx]
        src += ' # {} {} {}\n'.format(y,-x,z) # Axis conversion for RobotStudio

    done = False
    while done == False:
        num_steps += 1
        action = policy[ robot_mdp.MDP[3][next_state] ]
        #print(robot_mdp._ext2intAction(action))
        next_state, reward, done, info = robot_mdp._step(action)
        #state_rd   = robot_mdp.MDP[3][next_state]
        #next_state, reward, done, info = mdp_step(state_rd, action)
        #print(reward)

        # Register plan
        arms_pos, _, _, pick_pos = robot_mdp._ext2intState(next_state)
        idxs = []
        zs   = []
        grid_block = []
        gripper_action = []
        joint_a = robot_mdp._ext2intAction(action)
        for a_pos, p_pos, a_a in zip(arms_pos, pick_pos, joint_a):
            if p_pos > 0:
                tmp = (p_pos-1)//robot_mdp.P
                if a_a == 4:
                    x,y = robot_mdp.piecesLocation['start'][tmp]
                else:
                    x,y = robot_mdp.piecesLocation['end'][tmp]
                #print('5', x,y,p_pos)
                tmp_p_pos = ((p_pos-1)%robot_mdp.P) + 1
            else:
                x,y = a_pos
                tmp_p_pos = 0
            idxs.append(x + y*robot_mdp.M)
            # Go down, up, open, close gripper
            print('d', a_a, tmp_p_pos, robot_mdp.P)
            if tmp_p_pos == 0:
                if a_a == 4 or a_a == 5:                        gripper_action.append(GRIPPER_UP)
                else:                                           gripper_action.append(GRIPPER_XY)
            elif tmp_p_pos < (robot_mdp.P/2 + 1):               gripper_action.append(GRIPPER_DOWN)
            elif (tmp_p_pos == robot_mdp.P/2 + 1) and a_a == 4: gripper_action.append(GRIPPER_CLOSE) # pick
            elif (tmp_p_pos == robot_mdp.P/2 + 1):              gripper_action.append(GRIPPER_OPEN)  # drop
            else:                                               gripper_action.append(GRIPPER_UP)
            print('c', gripper_action)
            # Z value
            if tmp_p_pos == 0 and a_a != 4 and a_a !=5: zs.append(Z_PLANE);   grid_block.append(0)
            elif tmp_p_pos < (robot_mdp.P/2 + 1):       zs.append(Z_UPDOWN);  grid_block.append(1)
            elif tmp_p_pos == robot_mdp.P/2 + 1:        zs.append(Z_GRIPPER); grid_block.append(1)
            else:                                       zs.append(Z_UPDOWN);  grid_block.append(1)
            
        arms_config    = [list(robot.config[i, gblock, idx]) for i,(idx,gblock) in enumerate(zip(idxs, grid_block))]
        arms_loc       = [list(robot.location[idx]) for idx in idxs]
        #print(arms_loc)

        #for ang in reversed(arms_config):
        #    print(','.join([str(x) for x in ang]))

        #for pos,z in zip(reversed(arms_loc), reversed(zs)):
        #    x,y,_ = pos
        #    src += '{} {} {}\n'.format(y,-x,z) # Axis conversion for RobotStudio
        print('a')

        for ang,pos,z,gripper in zip(reversed(arms_config), reversed(arms_loc), reversed(zs), reversed(gripper_action)):
            # Format:
            #   ANGLES # GRIPPER (gripper open/close) or XYZ (MOVE Z)
            src += ','.join([str(x) for x in ang]) 
            if gripper == GRIPPER_OPEN:
                src += " # GRIPPER OPEN\n"
            elif gripper == GRIPPER_CLOSE:
                src += " # GRIPPER CLOSE\n"
            else:
                x,y,_ = pos
                src += ' # {} {} {}'.format(y,-x,z) # Axis conversion for RobotStudio
                if gripper == GRIPPER_UP:
                    src += " (GRIPPER UP)"
                elif gripper == GRIPPER_DOWN:
                    src += " (GRIPPER DOWN)"
                src += '\n'
        print('b')

    print('NUM_STEPS ', num_steps)
    
    print('\n')
    print('###############################################')
    print('# ADDITIONAL INFORMATION')
    print('#')
    print('# This plan is generated based on the next data')
    print('#')
    print('###############################################')
    print('')
    idxs           = [(x + y*robot.M) for x,y in initial_pos]
    arms_config    = [list(robot.config[i, 0, idx]) for i,idx in enumerate(idxs)]
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
        #pieces = [{'start' : [-350, 450, 0],  # Piece 1
        #           'end'   : [ 250, 250, 0],
        #          },                     
        #          {'start' : [-350, 300, 0],  # Piece 2
        #           'end'   : [ 150, 300, 0],
        #          },                     
        #          {'start' : [-50,  350, 0],  # Piece 3
        #           'end'   : [ 350, 400, 0],
        #          },                     
        #          {'start' : [-150, 450, 0],  # Piece 4
        #           'end'   : [ 250, 500, 0],
        #          }]
        # Pieces configuration
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


#    # Solve MDP
#    robot_mdp = mdp_generator(robot, pieces)
#
#    if not robot_mdp.load('MDP_RAW.bin'):
#        robot_mdp.generate()
#        robot_mdp.save('MDP_RAW.bin')
#
#    robot_mdp.update()
#
#    #                      Next states       Rewards
#    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
#    policy = solver.solve()
#
#    # Initial pos
#    init_pos   = [[1,2],[3,4]]
#    #init_pos   = [[1,4], [8,4]]
#    #init_pos   = [[3,0],[9,1]]
#    
#    # Show solution
#    #showSolution(policy, init_pos, 'robot.gif')
#    showSolution(policy, init_pos)

    # Test the solution for several pieces configurations and several arms init locations
    #
    np.random.seed(5)
    print("ArmsGridPos;Pieces location;Num steps")
    piecesTest = [pieces]
    num_pieces_cfg = 5
    num_arms_init_cfg = 5
    for j in range(num_pieces_cfg):
        # Set random pieces location
        #pieces = [{'start' : [-50 - 100*np.random.randint(5), 200 + 100*np.random.randint(5), 0],  # Piece 1
        #           'end'   : [ 50 + 100*np.random.randint(5), 200 + 100*np.random.randint(5), 0],
        #          } for _ in range(4)] # 4 pieces

        ## Trick to add pieces (more random locations would end up in some unreachable piece for both arms)
        #for kk in range(2):
        #   pieces.append(pieces[kk].copy())

        #Debug: Fixed pieces location
        pieces = [{'start' : [-350, 300, 0],  # Piece 1
                   'end'   : [ 250, 500, 0],
                  },                     
                  {'start' : [-250, 500, 0],  # Piece 2
                   'end'   : [  50, 200, 0],
                  },                     
                  {'start' : [-450, 300, 0],  # Piece 3
                   'end'   : [ 250, 500, 0],
                  },                     
                  {'start' : [-150, 600, 0],  # Piece 4
                   'end'   : [ 350, 200, 0],
                  }]
        #pieces[0] = pieces[3]
        #pieces[1] = pieces[2]
#        pieces[1]['start'] = [-450, 600, 0]
#        pieces[1]['end'] = [150, 500, 0]
#        print(pieces[1])
        #pieces[2] = pieces[3]
#        if j == 0:
#            for i in range(num_arms_init_cfg):
#                armsGridPos = [[np.random.randint(5),np.random.randint(5)], [np.random.randint(5,10),np.random.randint(5)]]
#            continue

        # Load MDP template update it with known pieces location
        # Dirty: like running the script on each iteration
#        if os.path.isfile('MDP_RAW.bin'):
#            os.remove('MDP_RAW.bin')
        #try:
        #    del robot_mdp
        #except:
        #    pass
        for i,piece in enumerate(pieces):
            z     = 180
            x1,y1,_ = piece['start']
            x2,y2,_ = piece['end']
            print("Piece %d: [%5d %5d %5d] -> [%5d %5d %5d]" % (i+1, y1, -x1, z, y2, -x2, z))
        
        if not j<0:
            # Create (or load) MDP template (no pieces info)
            # Then update template with pieces info
            robot_mdp = mdp_generator(robot, pieces)
            robot_mdp.update()
            print("MDP UPDATED")

            # Solve MDP
            solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
            policy = solver.solve()

        for i in range(num_arms_init_cfg):

            num_steps = 0
            # Set random arms init location
            armsGridPos = [[np.random.randint(5),np.random.randint(5)], [np.random.randint(5,10),np.random.randint(5)]]

            # Debug: Fixed arms position
            # [[-50, 500], [250, 600]] -> [[4, 3], [7, 4]] (left, right)
            armsGridPos = [[4, 3], [7, 4]]

            if j<0: continue


            # Run solution (get num of steps)
            #try:
            for _ in range(1):
                #showSolution(policy, armsGridPos)
                pieces_str = pieces if i==0 else "IDEM"
                print(robot_mdp.piecesLocation)
                print("DATA: ", armsGridPos, ";", pieces_str, ";", num_steps)
                generateTxtPlan(policy, armsGridPos, pieces)
                #import sys
                #sys.exit()
            #except:
            #    # Maybe there is no solution for the random pieces/arms init configuration
            #    import sys
            #    sys.exit()
            #    pass

            import sys
            sys.exit()
            break
#        # only the first pieces configuration
#        import sys
#        sys.exit()


    # Generate helper python code for applying a solution to YuMi robot
    # DMG: need to detect step related to a piece (there is no stored arm configuration for it. Pieces location does not match arm grid locations)
    #generatePythonPlan(policy, initial_pos, pieces)
    #generateTxtPlan(policy, init_pos, pieces)
