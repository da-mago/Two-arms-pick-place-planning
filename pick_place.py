import numpy as np
import time    # sleep
import imageio # mimsave
from mdp import mdp
from mdp_solver import mdp_solver
from yumi import YuMi

def showSolution(policy, GIF_filename=None):
    ''' Show pick & place solution '''
    next_state = robot_mdp._int2extState([[0,4],[8,4]],[0,0],15)
    robot_mdp.reset(next_state)
    images = []
    i = 0
    done = False
    while done == False:
        # Show pick&place solution
        robot_mdp.render()
        time.sleep(0.1)
        action = policy[ robot_mdp.MDP[3][next_state] ]
        next_state, reward, done, info = robot_mdp.step(action)

        # Show plan actions
        print(i, robot_mdp._ext2intAction(action))
        i += 1

        # Create GIF
        if not (GIF_filename == None):
            image = np.frombuffer(robot_mdp.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(robot_mdp.fig.canvas.get_width_height()[::-1] + (3,)) # Example: (640,480,3)
            images.append(image)
            if done == True:
                imageio.mimsave(GIF_filename, images, duration=0.2)
                

if __name__ == "__main__":

    # Robot
    robot = YuMi()

    # Pieces configuration
    pieces = [{'start' : [-0.5, -0.5, 0],  # Piece 1
               'end'   : [ 0.5, -1.5, 0],
              },
              {'start' : [-0.3, -0.8, 0],  # Piece 2
               'end'   : [ 1,   -1,   0],
              },
              {'start' : [-0.3, -1,   0],  # Piece 3
               'end'   : [ 0.3, -1,   0],
              },
              {'start' : [-0.6, -1.3, 0],  # Piece 4
               'end'   : [ 0.6, -0.8, 0],
              }]

    # Work area
    workArea = { 
               'size' : [10,5],                # XY grid cells
               'rect' : [-1.35, -0.2, 1.35, 1] # Left-top, bottom-right
               }

    # Solve MDP
    robot_mdp = mdp(robot, pieces, workArea)

    if not robot_mdp.load('MDP.bin'):
        robot_mdp.generate()
        robot_mdp.save('MDP.bin')

    robot_mdp.update()
    #                      Next states       Rewards
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
    policy = solver.solve()

    # Show solution
    showSolution(policy, 'robot.gif')
