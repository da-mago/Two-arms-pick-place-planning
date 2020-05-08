
from robot import Robot
import numpy as np

class Robot_2A2L(Robot):
    ''' Robot with two arms, and two links per arm '''

    def __init__(self):
        ''' Define the robot model and create an instance of Robot class '''

        # Robot model
        cfg = [ # Left arm
                {
                  'model'         : ['z', [1., 0., 0], 'z', [1., 0., 0.]], # Joints/Links
                  'modelPos'      : [-0.5, 0, 0],                          # Arm base coordinates
                  'defaultJoints' : [np.pi, 0],                            # Arm pose by default
                  'jointsSpeed'   : [180, 180]                             # Joint angular speed (degrees/s)
                },
                # Right arm
                {
                  'model'         : ['z', [1., 0., 0], 'z', [1., 0., 0.]],
                  'modelPos'      : [0.5, 0, 0], 
                  'defaultJoints' : [0, 0],
                  'jointsSpeed'   : [180, 180]
                }
              ]

        # Work area (2D grid definition)
        work_area = { 
                 'size' : [10,5],                  # XY grid cells
                 'rect' : [-1.35, -0.2, 1.35, 1.6] # Left-top, bottom-right
                    }

        super().__init__(cfg, work_area)

    def getNumArms(self):
        return len(self.robot)

