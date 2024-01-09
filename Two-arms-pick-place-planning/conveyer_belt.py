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

# Print colors
CRED    = '\033[91m'
CGREEN  = '\033[92m'
CORANGE = '\033[93m'
CEND    = '\033[0m'

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

##def DoSanityCheck(pieces_cfg, armsGridPos):
##
##    status = True
##
##    # Create robot instance (so implicity parsing excels content)
##    globalCfg = Cfg(1, 0, 100)
##    robot = Robot_YuMi(globalCfg)
##    # Create environment (just to convert Physical to grid pos)
##    env = env_pickplace(robot, pieces_cfg, globalCfg)
##
##    # Check Pieces
##    for i,piece in enumerate(pieces_cfg):
##        z       = 180
##        x1,y1,_ = piece['start']
##        x2,y2,_ = piece['end']
##        print("Piece %d: [%5d %5d %5d] -> [%5d %5d %5d] " % (i+1, y1, -x1, z, y2, -x2, z), end='')
##
##        # collect reachability info
##        reach = [[0,0], [0,0]]
##        for j in range(2): # each arm
##            for k,k_str in enumerate(['start', 'end']) :
####                pos = env.piecesLocation[k_str][i]
####                x,y,z = env._updatePosFromPiece(pos)
##                (x,y,z), _ = env._piece2robotPos(env.piecesLocation[k_str][i], 0, 0)
###                x,y,z = env.piecesLocation[k_str][i][:]
##                reach[j][k] = robot.reachable[j,x,y,z]
##
##        # check if the piece can be processed
##        for k, k_str in enumerate(['start', 'end']):     # start/end
##            if not reach[0][k] and not reach[1][k]:
##                print(CRED + "- (Invalid {}) ".format(k_str) + CEND, end='')
##                status = False
##
##        if env.T == 0: # No intermediate position
##            if (not reach[0][0] or not reach[0][1]) and \
##               (not reach[1][0] or not reach[1][1]):
##                print(CRED + "- (Invalid configuration)" + CEND, end='')
##                status = False
##        print()
##
##    # Check init position check
##    print()
##    for j in range(2):
##        x,y,z = armsGridPos[j]
##        print("Arm{} init: {}".format(j, robot.location[x,y,z]))
##        if not robot.reachable[j,x,y,z]:
##            print(CRED + "Invalid arm{} init position ".format(j) + CEND)
##            status = False
##
##    # Check intermediate position
##    if env.T: # No intermediate position
##        x,y,z = robot.T_pos
##        for j in range(0):
##            if not robot.reach[0,x,y,z]:
##                print(CRED + "Warning! Intermediate position not valid for arm{}".format(j) + CEND)
##                #status = False
##
##    if status:
##        print("\nSanity check: " + CGREEN + "PASS" + CEND)
##    else:
##        print("\nSanity check: " + CRED + "FAIL (abort)" + CEND)
##            
##    return status


if __name__ == "__main__":

    # EEs initial position
    armsGridPos = [[1, 4, 0], [8, 0, 0]]
    armsGridPos = [[6, 4, 0], [8, 0, 0]]
    
    # Pieces (config)
    pieces_cfg = [ {'start': [-750, 300, 180],'end'  : [-150, 600, 180]}, 
                   {'start': [-950, 400, 180],'end'  : [ -50, 600, 180]},
                   {'start': [-450, 500, 180],'end'  : [  50, 600, 180]},
                   {'start': [-650, 500, 180],'end'  : [ 150, 600, 180]} ]

    # Do basic sanity check on pieces and initial pos
##    status = DoSanityCheck(pieces_cfg, armsGridPos)

    # Project configuration
    num_pieces = 2
    num_layers = 1
    camera_X   = -70 # number of piece steps before reaching the grid
    action_mode = Cfg.ACTIONS_ORTHO_2D # ACTIONS_ORTHO_2D | ACTIONS_ORTHO_2D_DIAG_2D
    distance = 50
    algorithm = mdp_solver.ALG_VALUE_ITERATION # ALG_BFS | ALG_VALUE_ITERATION

    globalCfg = Cfg(num_layers, action_mode, distance)
    
    # Robot
    robot = Robot_YuMi(globalCfg)
    
    # Create output folder
    folder = 'output'
    if not os.path.exists(folder):
        os.mkdir(folder)

    # Generate offline MDP
    filename = "MDP_conveyor_pieces{}.bin".format(num_pieces)
    path = os.path.join(folder, filename)
    pieces = pieces_cfg[0:num_pieces]
    t0 = time.time()
    robot_mdp = mdp_generator(robot, pieces, globalCfg, path)
    t1 = time.time()
    time_offline = t1 - t0
    print("Time Offline: {}h {}m {}s".format(int(time_offline/3600), int((time_offline%3600)/60), int(time_offline%60)))

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
    # Note: first assumption, MDP computation takes 0 time
    #
    done = False
    while not done:

        done = True
        # Get the next piece detected by the camera

        # Simulate pieces scenario when new piece will be in tn1

        # Update pieces and reboto information for t=tn1

        # Online MDP update
        robot_mdp.update()

        f_reward, f_transition = robot_mdp.MDP[0:2]
        # (only for BFS algorithm)
        init_state = robot_mdp.MDP[3][ robot_mdp._int2extState(armsGridPos, [0,0], [1 for _ in range(robot_mdp.K)], [0,0], 0)  ]

        # Solve MDP
        t1 = time.time()
        solver = mdp_solver([f_reward, f_transition], init_state)
        policy, pathNactions = solver.solve(algorithm)
    
        if policy is not None:
            # Validate policy
            status = validatePolicy(policy, armsGridPos, pieces, robot_mdp)
        else:
            status = True
    
        t2 = time.time()

        # Update RobotStudio Plan
        if status:
            #            pieces_status = [1 for _ in range(robot_mdp.K)]
            #            for i in range(num_pieces, robot_mdp.K):
            #                pieces_status[i] = 0 # already processed
            pieces_status = []
            for i in range(robot_mdp.K):
                pieces_status.append(1 if i<num_pieces else 0)


            print("  Generating RobotStudio plan")
            if policy is not None:
                _, steps, plan = pick_place.generateRobotStudioInputFromPolicy(policy, armsGridPos, pieces, pieces_status, robot, robot_mdp)
            else:
                _, steps, plan = pick_place.generateRobotStudioInputFromPath(pathNactions, armsGridPos, pieces, pieces_status, robot, robot_mdp)
    
            filename = "RobotStudio_alg{}.txt".format(algorithm)
            path = os.path.join(folder, filename)
            with open(path, "w") as f:
                f.write(plan)
        
        time_online  = t2 - t1
        print("  Number of steps: {}".format(steps))
        print("  Time Online : {}h {}m {:0.3f}s".format(int(time_online/3600),  int((time_online%3600)/60),  time_online%60))

