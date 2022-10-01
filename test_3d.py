#
# Try random pieces configurations to get the ones using grid 3D
#

import numpy as np
import os
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver
from robot_YuMi import Robot_YuMi

def isUsing3DGrid(policy, initial_pos, pieces):

    # Set initial state
    pieces_status = [1 for _ in range(robot_mdp.K)]
    next_state = robot_mdp._int2extState(initial_pos, [0,0], pieces_status, [0,0])
    robot_mdp.reset(next_state)
    
    done = False
    while done == False:
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, _, done, _ = robot_mdp._step(action)

        joint_a = robot_mdp._ext2intAction(action)
        for a in joint_a:
            if a == robot_mdp.ACTION_UP:
                # At least, one arm is going up
                return True

    return False

def savePieces(pieces):
    with open('Pieces_3D.txt', 'a') as f:
        text = "pieces = ["
        for piece in pieces:
            text += "{'start' : %s,\n          " % (piece['start'])
            text += " 'end'   : %s,\n          " % (piece['end'])
            text += "},\n          "
        text += "]\n\n\n"
        f.write(text)

def getRandomPieces():
    # PENDING: randomize pieces pos
    pieces = [ 
              {'start' : [-250, 300, 0],  # Piece 1
               'end'   : [ 250, 500, 0],
              },                     
              {'start' : [-250, 500, 0],  # Piece 2
               'end'   : [ 350, 200, 0],
              },                     
#              {'start' : [-350, 300, 0],  # Piece 3
#               'end'   : [ 150, 500, 0],
#              },                     
#              {'start' : [-150, 300, 0],  # Piece 4
#               'end'   : [ 150, 600, 0],
#              }
              ]

    return pieces

def getRandomInitPos():
    # PENDING: randomize initial pos
    armsGridPos = [[6, 4, 0], [8, 0, 0]]

    return armsGridPos

#####################
## MAIN CODE       ##
#####################

# Robot
robot = Robot_YuMi()

while(True):

    # Get random pieces
    pieces = getRandomPieces()

    # Remove MDP file
    os.remove("MDP_RAW.bin")

    # Build MDP
    robot_mdp = mdp_generator(robot, pieces)
    robot_mdp.update()

    # Solve MDP
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
    policy = solver.solve()

    # The solution is valid for any init pos, so try some of them
    NUM_POS = 10
    for i in range(NUM_POS):
        armsGridPos = getRandomInitPos()
        if isUsing3DGrid(policy, armsGridPos, pieces):
            # Save pieces configuration to file
            savePieces(pieces)

    break

