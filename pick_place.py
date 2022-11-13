import os
import numpy as np
import time    # sleep
#import imageio # mimsave
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver
from env_pickplace import env_pickplace
from global_config import GlobalConfig as Cfg
from robot_YuMi import Robot_YuMi

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

    pieces_status = [1 for _ in range(robot.K)]
    init_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
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
        pos, status, pieces_status, pick_pos = robot_mdp._ext2intState(next_state)
        joint_a = robot_mdp._ext2intAction(action)
        print(joint_a, pos, ',', status, ',', pieces_status, ',', pick_pos, i)
        for arm_idx in range(1,-1,-1):
            x,y,z = pos[arm_idx]
            x,y,_ = robot_mdp.robot.location[x,y,z]
            #arm_config = robot.config[arm_idx, x,y,z]
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

    arms_config = [list(robot.config[i,x,y,z]) for i,(x,y,z) in enumerate(initial_pos)]
    arms_loc    = [list(robot.location[x,y,z]) for x,y,z in initial_pos]
    src += '# Robot init configuration\n'
    src += 'robot_init_cfg = {} # {} -> {}\n\n'.format(arms_config, initial_pos, arms_loc)

    src += 'robot_plan = [\n'
    src += '#   Configuration                                                                                                                               Actions  -  Grid location  -  Location\n'

    pieces_status = [1 for _ in range(robot_mdp.K)]
    next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
    robot_mdp.reset(next_state)
    done = False
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        previous_armsStatus = robot_mdp.armsStatus[:]
        next_state, reward, done, info = robot_mdp._step(action)

        # Register plan
        a_name =      [ 'Left            ', 'Right           ', 'Back            ', 'Front           ', 'Down            ', 'Up              ']
        if globalCfg.actions_mode != Cfg.ACTIONS_BASIC:
            a_name += [ 'Left_Front      ', 'Right_Front     ', 'Right_Back      ', 'Left_Back       ']

        if globalCfg.actions_mode == Cfg.ACTIONS_DIAGONAL_2D_3D:
            a_name += [ 'Up_Left         ', 'Up_Left_Front   ', 'Up_Front        ', 'Up_Right_Front  ',
                        'Up_Right        ', 'Up_Right_Back   ', 'Up_Back         ', 'Up_Left_Back    ',
                        'Down_Left       ', 'Down_Left_Front ', 'Down_Front      ', 'Down_Right_Front',
                        'Down_Right      ', 'Down_Right_Back ', 'Down_Back       ', 'Down_Left_Back  ']

        a_name +=     [ 'Pick            ', 'Drop            ', 'Stay            ']

        arms_action    = robot_mdp._ext2intAction(action)
        a_names        = [a_name[a] for a in arms_action]
        arms_pos, _, _, _ = robot_mdp._ext2intState(next_state)
        arms_config    = [list(robot.config[i,x,y,z]) for i,(x,y,z) in enumerate(arms_pos)]
        arms_loc       = [list(robot.location[x,y,z]) for x,y,z in arms_pos]

        arms_config = [] # remove arms pose info
        # Hack to add what piece
        for i,a in enumerate(arms_action):
            if a == robot_mdp.ACTION_PICK or a == robot_mdp.ACTION_DROP :
                p = robot_mdp.armsStatus[i] if a == robot_mdp.ACTION_PICK else previous_armsStatus[i]
                if robot_mdp.pickPos[i] > 0: piece = ((robot_mdp.pickPos[i] - 1) // robot_mdp.P) + 1
                else:                        piece = p
                a_names[i] += 'P{}'.format(piece) + ("I" if robot_mdp.armsGridPos[i] == robot_mdp.T_pos else " ")
            else:
                a_names[i] += '   '
        src += '    {},    \t# {},  {} -> {}\n'.format(arms_config, a_names, arms_pos, arms_loc)

    src += ']\n' # End of robot plan
    print(src)



def generateTxtPlan(policy, initial_pos, pieces, robot_mdp):

    # Sort of MACROs
    GRIPPER_XY    = 0
    GRIPPER_DOWN  = 1
    GRIPPER_CLOSE = 2
    GRIPPER_OPEN  = 3
    GRIPPER_UP    = 4
    #Z_PLANE       = 180
    #Z_UP          = 180
    #Z_DOWN        = 110
    #Z_GRIPPER     = Z_DOWN
    Z_GRIPPER     = [180, 110]

    src  = ''
    robot = robot_mdp.robot

    num_steps = 0
    pieces_status = [1 for _ in range(robot_mdp.K)]
    init_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
    next_state = init_state
    robot_mdp.reset(next_state)

    print("Generating plan ...")
    arms_config = [list(robot.config[i, x,y,z]) for i,(x,y,z) in enumerate(initial_pos)]
    #for ang in reversed(arms_config):
    #    print(','.join([str(x) for x in ang]))

    gripper = [0,0] # Gripper state: 0 (Open) | 1 (Close)
    for ang, pos, grip in zip(reversed(arms_config), reversed(initial_pos), reversed(gripper)):
        # Format:
        #   ANGLES # GRIPPER (gripper open/close) or XYZ (MOVE Z)
        src += ','.join([str(x) for x in ang]) 
        x,y,z = pos
        x,y,_ = robot.location[x,y,z]
        z = Z_GRIPPER[0]
        src += ',{} # {} {} {}\n'.format(grip, y,-x,z) # Axis conversion for RobotStudio

    done = False
    while done == False:
        num_steps += 1
        action = policy[ robot_mdp.MDP[3][next_state] ]
        #print(robot_mdp._ext2intAction(action))
        previous_next_state = next_state
        next_state, reward, done, info = robot_mdp._step(action)
        #state_rd   = robot_mdp.MDP[3][next_state]
        #next_state, reward, done, info = mdp_step(state_rd, action)
        #print(reward)

        # Register plan
        arms_pos, _, _, pick_pos = robot_mdp._ext2intState(next_state)
        poss = []
        zs   = []
        z_plane = []
        gripper_action = []
        joint_a = robot_mdp._ext2intAction(action)
        print(joint_a, robot_mdp.armsGridPos, robot_mdp.armsStatus, robot_mdp.piecesStatus, robot_mdp.pickPos)
        #gripper = [0, 0]
        for i, (a_pos, p_pos, a_a) in enumerate(zip(arms_pos, pick_pos, joint_a)):
            if p_pos > 0:
                tmp = (p_pos-1)//robot_mdp.P
                if a_a == robot_mdp.ACTION_PICK:
                    pos = robot_mdp.piecesLocation['start'][tmp]
                else:
                    pos = robot_mdp.piecesLocation['end'][tmp]
                #print('5', x,y,p_pos)
                tmp_p_pos = ((p_pos-1)%robot_mdp.P) + 1
            else:
                pos = a_pos
                tmp_p_pos = 0
            poss.append(pos)
            # Go down, up, open, close gripper
            #print('d', a_a, tmp_p_pos, robot_mdp.P)
            #                                                     opened/closed           Z                arm Zcomment (grip action)        
            if tmp_p_pos == 0:
                if a_a == robot_mdp.ACTION_PICK or a_a == robot_mdp.ACTION_DROP:   z_plane.append(0);  gripper_action.append(GRIPPER_UP)
                else:                                                              z_plane.append(0);  gripper_action.append(GRIPPER_XY)
            elif tmp_p_pos < (robot_mdp.P/2 + 1):                                  z_plane.append(1);  gripper_action.append(GRIPPER_DOWN)
            elif (tmp_p_pos == robot_mdp.P/2 + 1) and \
                  a_a == robot_mdp.ACTION_PICK:                   gripper[i] = 1;  z_plane.append(1);  gripper_action.append(GRIPPER_CLOSE)
            elif (tmp_p_pos == robot_mdp.P/2 + 1):                gripper[i] = 0;  z_plane.append(1);  gripper_action.append(GRIPPER_OPEN)
            else:                                                                  z_plane.append(1);  gripper_action.append(GRIPPER_UP)
            #print('c', gripper_action)
            # Z value
#            if tmp_p_pos == 0:
#                if a_a == robot_mdp.ACTION_PICK  or \
#                   a_a == robot_mdp.ACTION_DROP:        zs.append(Z_UP);      z_plane.append(1)
#                else:                                   zs.append(Z_PLANE);   z_plane.append(0)
#            if tmp_p_pos == 0 and \
#               a_a != robot_mdp.ACTION_PICK and \
#               a_a != robot_mdp.ACTION_DROP:            zs.append(Z_DOWN);    z_plane.append(0)
#            elif tmp_p_pos < (robot_mdp.P/2 + 1):       zs.append(Z_DOWN);    z_plane.append(1)
#            elif tmp_p_pos == robot_mdp.P/2 + 1:        zs.append(Z_GRIPPER); z_plane.append(1)
#            else:                                       zs.append(Z_UP);      z_plane.append(0)
            
        arms_config    = [list(robot.config[i, x,y,(z if gblock == 0 else robot.Z)]) for i,((x,y,z),gblock) in enumerate(zip(poss, z_plane))]
        arms_loc       = [list(robot.location[x,y,z]) for x,y,z in poss]
        #print(arms_loc)

        #for ang in reversed(arms_config):
        #    print(','.join([str(x) for x in ang]))

        #for pos,z in zip(reversed(arms_loc), reversed(zs)):
        #    x,y,_ = pos
        #    src += '{} {} {}\n'.format(y,-x,z) # Axis conversion for RobotStudio
        #print('a')

        for i,(ang, pos, z_p, grip_info, grip, a) in enumerate(zip(reversed(arms_config), reversed(arms_loc), reversed(z_plane), reversed(gripper_action), reversed(gripper), reversed(joint_a))):
            # Format:  ANGLES, grip state
            #if i==0: continue
            src += ','.join([str(x) for x in ang]) 
            src += ',{}'.format(grip) 
            # Add comment
            if grip_info == GRIPPER_OPEN:
                src += " # GRIPPER OPEN\n"
            elif grip_info == GRIPPER_CLOSE:
                src += " # GRIPPER CLOSE\n"
            else:
                x,y,_ = pos
                z     = Z_GRIPPER[z_p]
                src += ' # {} {} {}'.format(y,-x,z) # Axis conversion for RobotStudio
                extra_src = ' (Arm {} - {}) STATE {}\n'.format(i, "PICK" if a==4 else "DROP", previous_next_state)
                if grip_info == GRIPPER_UP:
                    src += " GRIPPER UP"
                    src += extra_src
                elif grip_info == GRIPPER_DOWN:
                    src += " GRIPPER DOWN"
                    src += extra_src
                else:
                    src +='\n'
       # print('b')

    print('NUM_STEPS ', num_steps)
    
    #print('\n')
    #print('###############################################')
    #print('# ADDITIONAL INFORMATION')
    #print('#')
    #print('# This plan is generated based on the next data')
    #print('#')
    #print('###############################################')
    #print('')
    arms_config = [list(robot.config[i, x,y,z]) for i,(x,y,z) in enumerate(initial_pos)]
    arms_loc    = [list(robot.location[x,y,z]) for x,y,z in initial_pos]
    #print('# Init robot location')
    #print('init_grid_pos = {} # Grid location'.format(initial_pos))
    #print('init_xyz_pos  = {} # XYZ location (EE)'.format(arms_loc))
    #print('init_ang_pos  = {} # Robot configuration'.format(arms_config))
    #print('# Copy & paste intial configuration')
    #for ang in arms_config:
    #    print(','.join([str(x) for x in ang]))
    #print('')
    #print('# Pieces configuration')
    #print('pieces_pos = {}'.format(pieces))

    #print(" # Plan based on EE locations (instead of joints)")
    print(src)
    
    return arms_pos, num_steps, src


if __name__ == "__main__":

    # User config
    globalCfg = Cfg(2, Cfg.ACTIONS_BASIC, 50)

    # Robot
    robot = Robot_YuMi(globalCfg)

    # EEs initial position
    armsGridPos = [[6, 4, 0], [8, 0, 0]]
    
    # Pieces
    pieces = [
              {'start' : [ -350, 200, 0],  # Piece 1
               'end'   : [ 450, 300, 0],
              },
              {'start' : [ 50,  600, 0],  # Piece 2
               'end'   : [-50,  200, 0],
              },
              {'start' : [ 250, 400, 0],  # Piece 3
               'end'   : [ -450, 200, 0],
              },
              {'start' : [ -50, 600, 0],  # Piece 4
               'end'   : [  50, 200, 0],
              }
             ]
    pieces = [ 
              {'start' : [-250, 300, 0],  # Piece 1
               'end'   : [ 250, 500, 0],
              },
              {'end' : [-250, 500, 0],  # Piece 2
               'start'   : [ 350, 200, 0],
              },                     
          ]

    # Dump pieces info
    for i,piece in enumerate(pieces):
        z       = 180
        x1,y1,_ = piece['start']
        x2,y2,_ = piece['end']
        print("Piece %d: [%5d %5d %5d] -> [%5d %5d %5d]" % (i+1, y1, -x1, z, y2, -x2, z))

    # Create (or load) MDP template (no pieces info)
    # Then update template with pieces info
    robot_mdp = mdp_generator(robot, pieces, globalCfg)
    algorithm_time = time.time()
    robot_mdp.update()
    print("\n\nMDP updated {}\n\n".format(time.time() - algorithm_time))

    # Solve MDP
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]], robot_mdp.single_nA)
    policy = solver.solve()
    print("\n\nAlgorithm execution {}\n\n".format(time.time() - algorithm_time))

    # Dump more info
    num_steps = 0
    print(robot_mdp.piecesLocation)
    print("DATA: ", armsGridPos, ";", pieces, ";", num_steps)
    generateTxtPlan(policy, armsGridPos, pieces, robot_mdp)
    generatePythonPlan(policy, armsGridPos, pieces)

