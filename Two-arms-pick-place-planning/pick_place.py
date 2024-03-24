import numpy as np
import os
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
    init_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0],0)
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
        pos, status, pieces_status, pick_pos, time_step = robot_mdp._ext2intState(next_state)
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
                

def generatePythonPlan(policy, initial_pos, pieces, robot, robot_mdp, globalCfg):

    src  = '# Pieces location\n'
    src += 'pieces_location = {}\n\n'.format(pieces)

    arms_config = [list(robot.config[i,x,y,z]) for i,(x,y,z) in enumerate(initial_pos)]
    arms_loc    = [list(robot.location[x,y,z]) for x,y,z in initial_pos]
    src += '# Robot init configuration\n'
    src += 'robot_init_cfg = {} # {} -> {}\n\n'.format(arms_config, initial_pos, arms_loc)

    src += 'robot_plan = [\n'
    src += '#   Configuration                                                                                                                               Actions  -  Grid location  -  Location\n'

    pieces_status = [1 for _ in range(robot_mdp.K)]
    next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0], 0)
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
        arms_pos, _, _, _, _ = robot_mdp._ext2intState(next_state)
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


def generateRobotStudioInputFromPath(pathNactions, initial_pos, pieces, pieces_status, robot_mdp):
    return generateTxtPlan(None, pathNactions, initial_pos, pieces, pieces_status, robot_mdp)


def generateRobotStudioInputFromPolicy(policy, initial_pos, pieces, pieces_status, robot_mdp):
    return generateTxtPlan(policy, None, initial_pos, pieces, pieces_status, robot_mdp)

def planStep(policy, pathNactions, state_idx_int, gripper, robot_mdp, num_steps):

    src = ''

    if policy is None:
        # Path is directly a list of states
        action = pathNactions[1][num_steps]
        state_idx_ext = pathNactions[0][num_steps]
        state_idx_int = robot_mdp.MDP[2][state_idx_ext]
        done = True if (num_steps >= (len(pathNactions[0]) - 1)) else False
    else:
        # Policy is action=f(state)
        state_idx_ext = robot_mdp.MDP[3][state_idx_int]
        action = policy[state_idx_ext]
        state_idx_int, _, done, _ = robot_mdp._step(action)

    arms_pos, _, _, pick_pos, time_step = robot_mdp._ext2intState(state_idx_int)
    poss = []
    zs   = []
    z_plane = []
    joint_a = robot_mdp._ext2intAction(action)

    for i, (a_pos, p_pos, a_a) in enumerate(zip(arms_pos, pick_pos, joint_a)):
        if p_pos > 0:
            tmp = (p_pos-1)//robot_mdp.P # piece
            ts_pickpos = ((p_pos-1)%robot_mdp.P) + 1 # pick_pos of piece
            if a_a == robot_mdp.ACTION_PICK:
                pos, _ = robot_mdp._piece2robotPos(robot_mdp.piecesLocation['start'][tmp], time_step, ts_pickpos)
            else:
                pos, _ = robot_mdp._piece2robotPos(robot_mdp.piecesLocation['end'][tmp], 0, 0)
            #print('5', x,y,p_pos)
            tmp_p_pos = ((p_pos-1)%robot_mdp.P) + 1
        else:
            pos = a_pos
            tmp_p_pos = 0
        poss.append(pos)
        # Go down, up, open, close gripper
        #                                                     opened/closed           Z                arm Zcomment (grip action)        
        if tmp_p_pos == 0:
            if a_a == robot_mdp.ACTION_PICK or a_a == robot_mdp.ACTION_DROP:   z_plane.append(0);
            else:                                                              z_plane.append(0);
        elif tmp_p_pos < (robot_mdp.P/2 + 1):                                  z_plane.append(1);
        elif (tmp_p_pos == robot_mdp.P/2 + 1) and \
              a_a == robot_mdp.ACTION_PICK:                   gripper[i] = 1;  z_plane.append(1);
        elif (tmp_p_pos == robot_mdp.P/2 + 1):                gripper[i] = 0;  z_plane.append(1);
        else:                                                                  z_plane.append(1);
        
    arms_config    = [list(robot_mdp.robot.config[i, x,y,(z if gblock == 0 else robot_mdp.robot.Z)]) for i,((x,y,z),gblock) in enumerate(zip(poss, z_plane))]

    for i,(ang, grip) in enumerate(zip(reversed(arms_config), reversed(gripper))):
        # Format:  ANGLES, grip state
        src += ','.join([str(x) for x in ang]) 
        src += ',{}\n'.format(grip) 

    return src, state_idx_int, gripper, done

def generateTxtPlan(policy, pathNactions, initial_pos, pieces, pieces_status, robot_mdp):

    src  = ''

    num_steps = 0
#    pieces_status = [1 for _ in range(robot_mdp.K)]
    state_idx_int = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0], 0)
    robot_mdp.reset(state_idx_int)
    robot = robot_mdp.robot

    arms_config = [list(robot.config[i, x,y,z]) for i,(x,y,z) in enumerate(initial_pos)]

    gripper = [0,0] # Gripper state: 0 (Open) | 1 (Close)
    for ang, grip in zip(reversed(arms_config), reversed(gripper)):
        # Format: ANGLES + GRIPPER (opened/closed)
        src += ','.join([str(x) for x in ang]) 
        src += ',{}\n'.format(grip)

    done = False
    gripper = [0,0] # Gripper state: 0 (Open) | 1 (Close)
    while done == False:

        src_step, state_idx_int, gripper, done = planStep(policy, pathNactions, state_idx_int, gripper, robot_mdp, num_steps)
        src += src_step
        num_steps += 1


##        if policy is None:
##            # Path is directly a list of states
##            action = pathNactions[1][num_steps]
##            state_idx_ext = pathNactions[0][num_steps]
##            state_idx_int = robot_mdp.MDP[2][state_idx_ext]
##            done = True if (num_steps >= (len(pathNactions[0]) - 1)) else False
##        else:
##            # Policy is action=f(state)
##            state_idx_ext = robot_mdp.MDP[3][state_idx_int]
##            action = policy[state_idx_ext]
##            state_idx_int, _, done, _ = robot_mdp._step(action)
##
##        num_steps += 1
##
##        arms_pos, _, _, pick_pos, time_step = robot_mdp._ext2intState(state_idx_int)
##        poss = []
##        zs   = []
##        z_plane = []
##        joint_a = robot_mdp._ext2intAction(action)
##
##        for i, (a_pos, p_pos, a_a) in enumerate(zip(arms_pos, pick_pos, joint_a)):
##            if p_pos > 0:
##                tmp = (p_pos-1)//robot_mdp.P # piece
##                ts_pickpos = ((p_pos-1)%robot_mdp.P) + 1 # pick_pos of piece
##                if a_a == robot_mdp.ACTION_PICK:
##                    pos, _ = robot_mdp._piece2robotPos(robot_mdp.piecesLocation['start'][tmp], time_step, ts_pickpos)
##                else:
##                    pos, _ = robot_mdp._piece2robotPos(robot_mdp.piecesLocation['end'][tmp], 0, 0)
##                #print('5', x,y,p_pos)
##                tmp_p_pos = ((p_pos-1)%robot_mdp.P) + 1
##            else:
##                pos = a_pos
##                tmp_p_pos = 0
##            poss.append(pos)
##            # Go down, up, open, close gripper
##            #                                                     opened/closed           Z                arm Zcomment (grip action)        
##            if tmp_p_pos == 0:
##                if a_a == robot_mdp.ACTION_PICK or a_a == robot_mdp.ACTION_DROP:   z_plane.append(0);
##                else:                                                              z_plane.append(0);
##            elif tmp_p_pos < (robot_mdp.P/2 + 1):                                  z_plane.append(1);
##            elif (tmp_p_pos == robot_mdp.P/2 + 1) and \
##                  a_a == robot_mdp.ACTION_PICK:                   gripper[i] = 1;  z_plane.append(1);
##            elif (tmp_p_pos == robot_mdp.P/2 + 1):                gripper[i] = 0;  z_plane.append(1);
##            else:                                                                  z_plane.append(1);
##            
##        arms_config    = [list(robot.config[i, x,y,(z if gblock == 0 else robot.Z)]) for i,((x,y,z),gblock) in enumerate(zip(poss, z_plane))]
##
##        for i,(ang, grip) in enumerate(zip(reversed(arms_config), reversed(gripper))):
##            # Format:  ANGLES, grip state
##            src += ','.join([str(x) for x in ang]) 
##            src += ',{}\n'.format(grip) 
    arms_pos_unused = 0
    return arms_pos_unused, num_steps, src


if __name__ == "__main__":

    # User config
    globalCfg = Cfg(1, Cfg.ACTIONS_BASIC, 50)

    # Robot
    robot = Robot_YuMi(globalCfg)

    # EEs initial position
    armsGridPos = [[6, 4, 0], [8, 0, 0]]
    
    # Pieces
    pieces = [
              {'start' : [-350, 200, 0],  # Piece 1
               'end'   : [ 450, 300, 0],
              },
              {'start' : [ 50,  600, 0],  # Piece 2
               'end'   : [-50,  200, 0],
              },
              {'start' : [ 250, 400, 0],  # Piece 3
               'end'   : [-450, 200, 0],
              },
              {'start' : [ -50, 600, 0],  # Piece 4
               'end'   : [  50, 200, 0],
              }
             ]
    #pieces = [ 
    #          {'start' : [-250, 300, 0],  # Piece 1
    #           'end'   : [ 250, 500, 0],
    #          },
    #          {'start' : [ 350, 200, 0],
    #           'end'   : [-250, 500, 0],  # Piece 2
    #          },                     
    #      ]

    # Dump pieces info
    for i,piece in enumerate(pieces):
        z       = 180
        x1,y1,_ = piece['start']
        x2,y2,_ = piece['end']
        print("Piece %d: [%5d %5d %5d] -> [%5d %5d %5d]" % (i+1, y1, -x1, z, y2, -x2, z))

    # Create (or load) MDP template (no pieces info)
    # Then update template with pieces info
    filename = "MDP_RAW.bin" 
    robot_mdp = mdp_generator(robot, pieces, globalCfg, filename)
    algorithm_time = time.time()
    robot_mdp.update()
    print("\n\nMDP updated {}\n\n".format(time.time() - algorithm_time))

    # Solve MDP
    init_state = robot_mdp.MDP[3][ robot_mdp._int2extState(armsGridPos, [0,0], [1 for _ in range(robot_mdp.K)], [0,0],0)  ]
    solver = mdp_solver(robot_mdp.MDP[0:2], init_state, 0)
    policy = solver.solve()
    print("\n\nAlgorithm execution {}\n\n".format(time.time() - algorithm_time))
    if isinstance(policy, list):
        for state in policy:
            int_state = robot_mdp.MDP[2][state]
            arms_pos, _, _, _, _ = robot_mdp._ext2intState(int_state)
            print(arms_pos)
        import sys
        sys.exit()

    # Dump more info
    num_steps = 0
    print(robot_mdp.piecesLocation)
    print("DATA: ", armsGridPos, ";", pieces, ";", num_steps)
    generateTxtPlan(policy, armsGridPos, pieces, robot, robot_mdp)
    generatePythonPlan(policy, armsGridPos, pieces, robot, robot_mdp, globalCfg)

