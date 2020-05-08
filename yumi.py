###############################################################################
#
# Model containing required YuMi stuff for solving pick&place problem:
# - LUTs (excel format):
#   - collisions (including minimun distance between arms)
#   - arms position reachability
#   - grid arms configuration (angles)
#
# - Methods:
#   - isThereCollision()
#   - isReachable()
#   - plot()
#
###############################################################################

import numpy as np
#import pandas as pd
from robot import Robot

class YuMi(Robot):
    ''' YuMi (14 degrees of freedom) robot. '''

    def __init__(self):
        ''' Load Robot data from the spreadsheet '''
        LUT_min_distance = np.zeros((10,5,10,5)) # Minimum distance for each robot configuration (M*N*M*N)
        LUT_reachability = np.zeros((2,10,5))    # For each arm (2), ability to get each grid position (M*N)
        LUT_configuration = np.zeros((2,10,5,7)) # For each arm (2), configuration (7 angles) data for each grid position (M*N)

        self.collision_distance = 0.5
        #self.work_area = ...

    def checkCollision(self, armsGridPos):
        ''' Check Robot collision.

            Note: any distance lower than 'collision_distance' is also 
                  considered collision.
        '''
        pos_tuple = ()
        for pos in armsGridPos:
            x,y = pos
            pos_tuple += (x,y,)

        return LUT_min_distance[pos_tuple] > self.collision_distance


    def checkReachability(self, armsGridpos):
        ''' Check that both arms can reach their target positions '''
        for i,(x,y) in enumerate(armsGridPos):
            if LUT_reachability[i,x,y]:
                return False

        return True


    def plot(self, armsGridPos, ax, plt3d):
        ''' Draw only EEs position (just a circle) '''
        for col,(x,y) in zip(['r', 'b'], armsGridPos):
            if plt3d: ax.plot3D(x, y, z, c=col, marker='o')
            else:     ax.plot  (x, y,    c=col, marker='o')


if __name__ == "__main__":

    # Robot
    robot = YuMi()
