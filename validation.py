# Validation
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver
from env_pickplace import env_pickplace
from global_config import GlobalConfig as Cfg
from robot_YuMi import Robot_YuMi
import os
import pick_place

def validate(policy, initial_pos, pieces, robot_mdp):

    pieces_status = [1 for _ in range(robot_mdp.K)]
    next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
    robot_mdp.reset(next_state)

    done = False
    max_iterations = 100
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp._step(action)
        max_iterations -= 1
        if max_iterations <= 0:
            break

    CRED   = '\033[91m'
    CGREEN = '\033[92m'
    CEND   = '\033[0m'
    if done: print("Validating: " + CGREEN + "PASS" + CEND)
    else   : print("Validating: " + CRED   + "FAIL" + CEND)

    return done

# EEs initial position
armsGridPos = [[6, 4, 0], [8, 0, 0]]

# Pieces
pieces_cfg = [
          {'start' : [-350, 200, 0],  # Piece 1
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
pieces_cfg = [ 
          {'start' : [-250, 300, 0],  # Piece 1
           'end'   : [ 250, 500, 0],
          },
          {'start' : [ 350, 200, 0],
           'end'   : [-250, 500, 0],  # Piece 2
          },                     
      ]

for num_layers in [1,2,3]:
    for action_mode in [0,1,2]:
        for num_pieces in [2, 4]:
            for distance in [50]:

                filename = "data_d{}_pieces{}_amode{}_layers{}.bin".format(distance, num_pieces, action_mode, num_layers)
                print("\nGenerating '{}'".format(filename))
                #if os.path.exists(filename):
                #    os.remove(filename)

                # User config
                globalCfg = Cfg(num_layers, action_mode, distance)
                
                # Robot
                robot = Robot_YuMi(globalCfg)

                pieces = pieces_cfg[0: num_pieces]
                robot_mdp = mdp_generator(robot, pieces, globalCfg, filename)
                robot_mdp.update()

                #f_reward, f_transition = robot_mdp.MDP[0:2]
                #solver = mdp_solver([f_reward, f_transition], robot_mdp.single_nA)
                solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]], robot_mdp.single_nA)
                policy = solver.solve()

                #pick_place.generateTxtPlan(policy, armsGridPos, pieces, robot, robot_mdp)
                #pick_place.generatePythonPlan(policy, armsGridPos, pieces, robot, robot_mdp, globalCfg)

                validate(policy, armsGridPos, pieces, robot_mdp)

                del robot
                del robot_mdp
                del solver
                del policy

