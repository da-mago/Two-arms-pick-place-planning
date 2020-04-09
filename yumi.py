
from robot import Robot
import numpy as np

class YuMi(Robot):

    def __init__(self):
        ''' Define the robot model and create an instance of Robot class '''
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

        super().__init__(cfg)

    def getNumArms(self):
        return len(self.robot)

