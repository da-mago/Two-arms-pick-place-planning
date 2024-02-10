# TODO:
# - if an MDP is regenerated with pick_pos > 0...this is not well supported

# Validation
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver
from env_pickplace import env_pickplace
from global_config import GlobalConfig as Cfg
from robot_YuMi import Robot_YuMi
import os
import pick_place
import shutil
import time
import copy

# Print colors
CRED     = '\033[31m'
CGREEN   = '\033[32m'
CORANGE  = '\033[33m'
CPURPLE  = '\033[34m'
CPINK    = '\033[35m'
CBLUE    = '\033[36m'
CEND     = '\033[0m'
BCRED    = '\033[41m'
BCGREEN  = '\033[42m'
BCORANGE = '\033[43m'
BCPURPLE = '\033[44m'


def print_DEBUG(text):
#    print(text)
    pass


def validatePolicy(policy, initial_pos, pieces, robot_mdp):

    pieces_status = [1 for _ in range(robot_mdp.K)]
    next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0], 0)
    #next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
    robot_mdp.reset(next_state)

    done = False
    max_iterations = 100
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        print(robot_mdp._ext2intState(next_state))
        next_state, reward, done, info = robot_mdp._step(action)
        max_iterations -= 1
        if max_iterations <= 0:
            break

    if done: print("  Validating: " + CGREEN + "PASS" + CEND)
    else   : print("  Validating: " + CRED   + "FAIL" + CEND)

    return done

def getPlan(robot, max_pieces, pieces, state_vars, globalCfg, mdp_path):

    # Fill up to max_pieces
#    fake_pieces = copy.deepcopy(pieces)
#    while len(fake_pieces) < max_pieces:
#        fake_pieces.append(pieces[0])

    # Load/generate offline MDP
#    robot_mdp = mdp_generator(robot, fake_pieces, globalCfg, mdp_path)
    robot_mdp = mdp_generator(robot, pieces, globalCfg, mdp_path)

    # Update MDP with pieces information
#    robot_mdp.update(pieces)
    robot_mdp.update()

    # Get plan
    f_reward, f_transition, intState, extState = robot_mdp.MDP[0:4]
    int_state = robot_mdp._int2extState(*state_vars)
    state = extState[int_state]
    solver = mdp_solver([f_reward, f_transition], state, algorithm)
    policy, plan = solver.solve()

    # If MDP approach, convert the policy to a plan (sequence of states)
    if policy is not None:
        max_iterations = 100
        plan = []
        robot_mdp.reset(int_state)
        done = False
        while done == False:
            int_state, _, done, _ = robot_mdp._step(policy[state])
            state = extState[int_state]
            plan.append(state)
            #int_state = f_transition[int_state][ policy[int_state] ]
            max_iterations -= 1
            if max_iterations <= 0:
                raise Exception("Plan not found")

    return plan, robot_mdp
    
def fillImageBuffer(time_step, undetected_pieces, detected_pieces, mdp_pieces, int_state):
    global x,y

    compensation = -1
    time_step = int_state[4]
    # empty screen
    l = 90
    screen = [[' ' for _ in range(l+1)] for _ in range(5)]
    colors =  [[0 for _ in range(l+1)] for _ in range(5)]
    color_type = [CRED, CGREEN, CORANGE, CPURPLE, CPINK, CBLUE, CEND, BCRED, BCGREEN, BCORANGE, BCPURPLE]

    # add mdp pieces
    arms_status = int_state[1]
    pieces_status = int_state[2]
    pick_pos = int_state[3]
    hide = [((p-1)//mdp.P) for p in pick_pos if p>0]
    for i,p in enumerate(mdp_pieces):
        # consider pieces in pick_pos as already picked up
        if pieces_status[i] == 1 and i not in hide:
            x = l - ((450 - p['start'][0]) // 20) + compensation
            y = (p['start'][1] - 200) // 100
            screen[y][x + time_step] = 'o'
            colors[y][x + time_step] = (150 - p['end'][0]) // 100

    # add detected pieces
    for p in detected_pieces:
        x = l - ((450 - p['start'][0]) // 20) + compensation
        y = (p['start'][1] - 200) // 100
        screen[y][x] = '*'
        colors[y][x] = (150 - p['end'][0]) // 100

    # add non-detected pieces
    for p in undetected_pieces:
        x = l - ((450 - p['start'][0]) // 20) + compensation
        if x < l and x >=0: # only pieces inside screen range
            y = (p['start'][1] - 200) // 100
            screen[y][x] = '>'
            colors[y][x] = 6

    # add containers
    for i,xmm in enumerate([150,50,-50,-150]):
        x = l - ((450 - xmm) // 20)
        y = 4
        screen[y][x] = ' '
        for j in range(3):
            colors[y][x-1+j] = 7+i
        colors[y][x+2] = 6

    # add robots
    arms_pos = int_state[0]
    for a,s in zip(arms_pos, arms_status):
        x = l - a[0]*5
        y = a[1]
        screen[y][x] = 'O'
        colors[y][x] = 6 if s == 0 else (150 - (mdp_pieces[s-1]['end'][0])) // 100

    return screen, colors

def showImageBuffer(total_t, time_step, plan_idx, camera_pos, screen, colors):
    color_type = [CRED, CGREEN, CORANGE, CPURPLE, CPINK, CBLUE, CEND, BCRED, BCGREEN, BCORANGE, BCPURPLE]
    # Show
    l = 90
    x = (1350 + camera_pos) // 20
    print("t = {}, plan_t = {}, n_plan: {}".format(total_t, time_step, plan_idx if plan_idx>0 else 'None'))
    print()
    print(' '*(x+1) + 'C' + ' '*(42-x) + "-450 -350 -250 -150  -50   50  150  250  350  450")
    print(CEND + ' ' + '|    '*18 + '|')
    print(CEND + '-'*(l+3))
    for i,s in enumerate(screen):
        print(CEND + '|', end='')
        for j,c in enumerate(s):
            print(color_type[colors[i][j]] + c, end='');
        print(CEND + '|')
    print(CEND + '-'*(l+3))


    # Move pointer to the screen top-left
    num_lines = (6 + len(screen))
    for i in range(num_lines):
        print('\033[F', end='')

    return num_lines

def showPlans(t, offset, plan1, plan2):
    ''' Debug purposes '''

    # Skip offset lines
    for i in range(offset):
        print()

    print(f"\n| {'Current plan':60} | {'Next plan':60} |")
    print(   "| {:60} | {:60} |".format("_"*60, "_"*60))
    extState = mdp.MDP[2]
    num_lines = 2
    if plan1 != None:
        for j in range(t):
            print("| {:60} | {:60} |".format(str(mdp._ext2intState(extState[ plan1[0][j] ])), ""))
        num_lines += t
    if plan2 != None:
        for j in range(len(plan2[0])):
            text1 = str(mdp._ext2intState(extState[ plan1[0][t+j] ])) if plan1 != None and (j+t) < len(plan1[0]) else ""
            text2 = str(mdp._ext2intState(extState[ plan2[0][j]   ]))
            print("| {:60} | {:60} |".format(text1, text2))
        num_lines += len(plan2[0])

#    # Got to 0,0
#    for i in range(offset + num_lines):
#        print("\033[F", end='')
    print()
    print()

    return offset + num_lines

    
def reproduce(imageSequence):
    t = 0
    for i,(screen, colors) in enumerate(imageSequence):
        # Info is not group per plan, so plan_t is unknonw
        showImageBuffer(t, 0, i, camera_pos, screen, colors)
        time.sleep(0.5)
        t += 1

def state2RobotStudio(mdp, int_state, int_action):

    arms_pos, _, _, pick_pos, _ = int_state
    gripper = [2,2] # No gripper action (keep the same status, opened or closed)
    arms_cfg = []

    for i in range(2):
        #DMG la accion solo esta en el plan de BFS de momento
        x,y,z = arms_pos[i]
        if mdp == None:
            if pick_pos[i] > 0: raise Exception("no pot ser este pick pos {}".format(int_state))
        else:
            tmp = 0 if pick_pos[i]==0 else ((pick_pos[i]-1) % mdp.P) + 1
            if tmp > 0:
                z = robot.Z # piece pick/drop height
                if tmp == (mdp.P/2 + 1): # moment where the gripper is closed or opened
                    gripper[i] = 1 if int_action[i] == mdp.ACTION_PICK else 0
                    if int_action[i] != mdp.ACTION_PICK and int_action[i] != mdp.ACTION_DROP: raise Exception("no pot ser {}".format(action))

        arms_cfg.append( list(robot.config[i, x,y,z]) )

    return arms_cfg, gripper

if __name__ == "__main__":

    x,y=0,0

    # EEs initial position
    armsGridPos = [[1, 4, 0], [8, 0, 0]]
    armsGridPos = [[6, 4, 0], [8, 0, 0]]
    
    # Pre-define the initial location of all pieces involved in the task (ordered from closer to farther)
    all_pieces = [ {'start': [ -850, 300, 180],'end'  : [-150, 600, 180]}, 
                   {'start': [ -950, 400, 180],'end'  : [ -50, 600, 180]},
                   {'start': [-1050, 500, 180],'end'  : [ 150, 600, 180]},
                   {'start': [-1150, 500, 180],'end'  : [ 150, 600, 180]},
                   {'start': [-1250, 500, 180],'end'  : [  50, 600, 180]},
                   {'start': [-1350, 400, 180],'end'  : [ -50, 600, 180]},
                   {'start': [-1450, 300, 180],'end'  : [-150, 600, 180]}, 
                   {'start': [-1550, 500, 180],'end'  : [ 150, 600, 180]},
                   {'start': [-1650, 500, 180],'end'  : [  50, 600, 180]},
                   {'start': [-1750, 400, 180],'end'  : [ -50, 600, 180]},
                   {'start': [-1850, 300, 180],'end'  : [-150, 600, 180]} ]
#    all_pieces = all_pieces[:9]
    # Project configuration
    max_mdp_pieces = 4
    grid_layers = 1
    action_mode = Cfg.ACTIONS_ORTHO_2D # ACTIONS_ORTHO_2D | ACTIONS_ORTHO_2D_DIAG_2D
    distance = 50
    algorithm = mdp_solver.ALG_BFS # ALG_BFS | ALG_DIJKSTRA |ALG_VALUE_ITERATION
    camera_pos   = -850 # camera detection point (X axis in mm)

    globalCfg = Cfg(grid_layers, action_mode, distance, algorithm)
    
    # Create output folder
    folder = 'output'
    if not os.path.exists(folder):
        os.mkdir(folder)

    # Robot
    robot = Robot_YuMi(globalCfg)
    
##    # Generate offline MDP
##    filename = "MDP_conveyor_pieces{}.bin".format(mdp_pieces)
##    path = os.path.join(folder, filename)
##    # Podemos evitar pasar esto al generador si no lo va usar?
##    pieces = all_pieces[0:mdp_pieces] # Arbitrary location is valid for offline policy generation
##    t0 = time.time()
##    robot_mdp = mdp_generator(robot, pieces, globalCfg, path)
##    t1 = time.time()
##    time_offline = t1 - t0
##    print("Time Offline: {}h {}m {}s".format(int(time_offline/3600), int((time_offline%3600)/60), int(time_offline%60)))
##
    # Setup example
    #
    # t =         t0    tn1       tn2
    # (X axis)  camera   |        MDP       |
    #              |     |         |  Grid  |
    #              C     ..........xxxxxxxxxx 
    #  1 2   4     1 
    #    4     3 2
    #    1 2    4
    # 
    # iAssumptions:
    # - Piece detection is done by another process/task (out of scope). This task feeds the list
    #   of pieces detected (and not yet processed)

    done = False
    piece_idx = 0
    total_t = 0
    t = 0 # time_step
    next_plan_t = -0xFFFF # Arbitraty big time_step
    plan = None
    next_plan = None
    time_step = 0
    detected_pieces = []
    mdp_pieces = [all_pieces[0] for _ in range(max_mdp_pieces)] # Fill with arbitrary info for the pieces
    filename = "MDP_conveyor_pieces{}.bin".format(max_mdp_pieces)
    mdp_path = os.path.join(folder, filename)
    int_state = [armsGridPos, [0,0], [0 for _ in range(max_mdp_pieces)], [0,0], 0]
    cnt = 0
    image_sequence = []
    offset = 0
    plan_idx = 0
    undetected_idx = 0
    gripper = [0,0]
    joint_plan = ''
    mdp = None

    arms_cfg, _ = state2RobotStudio(None, int_state, [0,0])
    joint_plan += ',{}\n'.format(int_state) 
    for a,b in zip(arms_cfg[::-1], gripper[::-1]): # For no reason, RobotStudio file use the reverse format
        joint_plan += ','.join([str(x) for x in a]) + ',{}\n'.format(b) 

    print()
    print()
    while not done:

        # Is there any new piece/s detected? If so, update detected_pieces list
        new_detected = [piece for piece in all_pieces if piece['start'][0] == (camera_pos + 20)]
        detected_pieces.extend(new_detected)
        undetected_idx += len(new_detected)
#        print('detected: ', detected_pieces)
        print_DEBUG('t={}, detected={}, is_plan={}'.format(t, len(detected_pieces), plan != None))

        # can one or more detected pieces be incorporated to the MDP? If so, regenerate the plan
        regenerate_plan = False
        if len(detected_pieces) > 0:
            pieces_status = int_state[2]
            arms_status   = int_state[1]
            if plan == None:
                regenerate_plan = True
            else:
                for i in range(max_mdp_pieces):
                    if pieces_status[i] == 0 and (i+1) not in arms_status:
                        regenerate_plan = True
                        break

        if regenerate_plan:
            # Select the pieces for the new plan
            for i in range(max_mdp_pieces):
                if pieces_status[i] == 1:
                    # keep piece still moving in the conveyor belt. Update its current location
                    mdp_pieces[i]['start'][0] += 20*t
                elif (i+1) in arms_status:
                    # keep piece being carried out by an arm. Do nothing
                    pass
                else:
                    if len(detected_pieces) > 0:
                        # incorporate piece from detected list
                        mdp_pieces[i] = copy.deepcopy(detected_pieces.pop(0))
                        pieces_status[i] = 1

            # Reset time_step
            int_state[4] = 0

        # Show environment
        screen, colors = fillImageBuffer(t, all_pieces[undetected_idx:], detected_pieces, mdp_pieces, int_state)
        offset = showImageBuffer(total_t, int_state[4], plan_idx, camera_pos, screen, colors)
        # just store to reproduce the solution
        image_sequence.append([screen, colors])

        if regenerate_plan:
            # Compute the next plan
            int_state[4] = 0

#            screen, colors = fillImageBuffer(0, all_pieces[undetected_idx:], detected_pieces, mdp_pieces, int_state)
#            offset = showImageBuffer(total_t, t, plan_idx, camera_pos, screen, colors)

            if True:
                for i in range(offset): print()
#                print("\033[{}BComputing new plan\033[{}A".format(offset, offset + 1))
                print("Computing new plan")
                for i in range(max_mdp_pieces):
                    if pieces_status[i] == 1 or (i+1) in arms_status:
                        print(mdp_pieces[i])
                print(int_state)
            next_plan, mdp = getPlan(robot, max_mdp_pieces, mdp_pieces, int_state, globalCfg, mdp_path)
            if True:
                print(mdp._ext2intState(mdp.MDP[2][next_plan[0][0]]))
            next_plan_t = 0
            plan_idx += 1

        if next_plan_t == 0:
            showPlans(t, offset, plan, next_plan)

            del plan
            plan = next_plan
            next_plan = None
            t = 0
            next_plan_t = -0xFFFF
            print_DEBUG("start next plan")

        #raise Exception()
        # Next time_step according to the current plan
        if plan and t >= len(plan[0]):
            plan = None

        if plan:
            print_DEBUG('Plan step {} {}'.format(t, plan[0][t]))
            state, action = plan[0][t], plan[1][t]
            int_state = list(mdp._ext2intState(mdp.MDP[2][state]))
            int_action = mdp._ext2intAction(action)
        else:
            int_action = [0,0]

        joint_plan += ',{}\n'.format(int_state) 
        arms_cfg, gripper_action = state2RobotStudio(mdp, int_state, int_action)
        for i,g_action in enumerate(gripper_action):
            if g_action < 2: gripper[i] = g_action
        for a,b in zip(arms_cfg[::-1], gripper[::-1]): # For no reason, RobotStudio file use the reverse format
            joint_plan += ','.join([str(x) for x in a]) + ',{}\n'.format(b) 
#                    data.append(

#                # Convert state to joint configuration
#                arms_pos, _, _, pick_pos, _ = int_state
#                data = []
#                for i in range(2):
#                    #DMG la accion solo esta en el plan de BFS de momento
#                    action = plan[1][t]
#                    x,y,z = arms_pos[i]
#                    tmp = 0 if pick_pos[i]==0 else ((pick_pos[i]-1)%mdp.P) + 1
#                    if tmp > 0:
#                        z = robot.Z # table plane
#                        if tmp == (mdp.P/2 + 1): # moment where the gripper is closed or opened
#                            gripper[i] = 1 if action == mdp.ACTION_PICK else 0
#
#                    arm_config = list(robot.config[i, x,y,z])
#                    # Format:  joint configuration and gripper state
#                    data.append(','.join([str(x) for x in arm_config]) + ',{}\n'.format(gripper[i])) 
#                joint_plan += "{}\n".format(int_state)
#                for d in data[::-1]: # For no reason, RobotStudio file use the reverse format
#                    joint_plan += d

                #print(src[1])
                #print(src[0])
                #print("\033[F", end ='') 
                #print("\033[F", end ='') 
                            #                arms_config = [list(robot.config[i, x,y,(z if pp == 0 else robot.Z)]) for i,((x,y,z), pp) in enumera
                            #
                            #                src = ''
                            #                for i,(ang, grip) in enumerate(zip(reversed(arms_config), reversed(gripper))):
                            #                    # Format:  ANGLES, gripper state
                            #                    src += ','.join([str(x) for x in ang]) 
                            #                    src += ',{}\n'.format(grip) 


        # Prepare for the next time_step
        total_t += 1
        t += 1
        next_plan_t += 1
        for p in all_pieces: # dark point: detected_pieces is updated as well since it is sharing pieces with all_pieces
            p['start'][0] += 20

#        # Show environment
#        screen, colors = fillImageBuffer(t, all_pieces[undetected_idx:], detected_pieces, mdp_pieces, int_state)
#        offset = showImageBuffer(total_t, t, plan_idx, camera_pos, screen, colors)
#        # just store to reproduce the solution
#        image_sequence.append([screen, colors])

        time.sleep(0.2)

        if plan == None and next_plan == None and undetected_idx >= len(all_pieces):
            done = True

    # Create RobotStudio input file
    filename = "RobotStudio_alg{}.txt".format(algorithm)
    path = os.path.join(folder, filename)
    with open(path, "w") as f:
         f.write(joint_plan)
#        cnt += 1;
#        if cnt > 80: break

##
##        
##        # 1. Prepare the next MDP
##        # 1.1 Select MDP pieces (up to mdp_pieces)
##        pieces = []
##        while piece_idx < len(all_pieces):
##            if (len(pieces) >= mdp_pieces) or \
##               ((all_pieces[piece_idx]['start'][0] + time_step*20) < camera_pos):
##               break
##            pieces.append(all_pieces[piece_idx])
##            piece_idx += 1
##
##        # 1.2. Update MDP
##        robot_mdp.update(pieces)
##
##        # 2. Solve MDP
##        pieces_status = [1 if i<len(pieces) else 0 for i in range(robot_mdp.K)]
##        init_state = robot_mdp.MDP[3][ robot_mdp._int2extState(armsGridPos, armsStatus, pieces_status, pick_pos, 0) ] # (only for BFS algorithm)
##
##        t1 = time.time()
##        f_reward, f_transition = robot_mdp.MDP[0:2]
##        solver = mdp_solver([f_reward, f_transition], init_state)
##        policy, pathNactions = solver.solve(algorithm)
##        t2 = time.time()
##    
##        # 3. Simulate (stop simulation with the first processed piece)
##        # How much time_steps??
##        if len(pieces) >= mdp_pieces:
##            time_step += returned_from_simulation()
##        else:
##            # Simulating less than K pieces on an K-pieces MDP
##            # There is the chance that a new piece is detected before the first piece is processed, so better to compute a new MDP from this point on.
##            if PIECE_DETECTED_BEFORE_PIECE_IS_PROCESSED:
##                ADD_DETECTED_PIECE_up_to_mdp_pieces
##                time_step += get_from_this_new_piece_WHICH_IS_BEFORE_returned_from_simulation()
##
##
##        # 4. Prepare next iterarion
##        # WHICH armpos, armsStatus, piecesStatus, ...
##
##        if policy is not None:
##            # Validate policy
##            status = validatePolicy(policy, armsGridPos, pieces, robot_mdp)
##        else:
##            status = True
##    
##
##        # Update RobotStudio Plan
##        if status:
##            print("  Generating RobotStudio plan")
##            if policy is not None:
##                _, steps, plan = pick_place.generateRobotStudioInputFromPolicy(policy, armsGridPos, pieces, pieces_status, robot_mdp)
##            else:
##                _, steps, plan = pick_place.generateRobotStudioInputFromPath(pathNactions, armsGridPos, pieces, pieces_status, robot_mdp)
##    
##            filename = "RobotStudio_alg{}.txt".format(algorithm)
##            path = os.path.join(folder, filename)
##            with open(path, "w") as f:
##                f.write(plan)
##        
##        time_online  = t2 - t1
##        print("  Number of steps: {}".format(steps))
##        print("  Time Online : {}h {}m {:0.3f}s".format(int(time_online/3600),  int((time_online%3600)/60),  time_online%60))

