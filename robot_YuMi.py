
from robot import Robot
import numpy as np
import csv

class Robot_YuMi():
    ''' YuMi robot (two arms, each 7 links) 
        No forward/inverse kinematicis.
        Grid definition, arm configuration for each grid point, collision
        check between arms fo any available position/arm, ... is precomputed
        outside this entity and stored, probably in a csv file, which will 
        be read by this class.
    '''

    def __init__(self, min_dist=60, z=180):
        ''' Define the robot model and create an instance of Robot class '''

        self.min_dist = min_dist # Distance considered collision 
        self.z = z               # Default z value for robot EEs

        self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision.csv', 'AlcanceDef1.csv')    


    def _loadCSV(self, distanceFilename, configFilename):
        ''' Read and process robot data from csv file '''

        # Read external data
        #
        # 1. Robot configuration
        with open(configFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            csv_data = [row for i,row in enumerate(csv_reader) if i>=2]

        grid_size = len(csv_data)
        angles_size = 7

        reachable = np.zeros((2, grid_size), dtype=np.uint8)
        config    = np.zeros((2, grid_size, angles_size), dtype=np.float) 
        location  = []
        x,y = [], []
        for i,row in enumerate(csv_data):
            location.append([int(row[1]), int(row[0]), self.z]) # xyz
            reachable[0, i] = 1 if row[2 ]=='Si' else 0         # valid EE location
            reachable[1, i] = 1 if row[10]=='Si' else 0
            config[0, i] = [float(t) for t in row[3:10]]        # 7 angles
            config[1, i] = [float(t) for t in row[11:18]]
            y.append(int(row[0]))
            x.append(int(row[1]))
        location = np.array(location, dtype=np.int16)
        M = len(set(x))
        N = len(set(y))
        print(M,N)

        # 2. Robot collision (distance between arms)
        with open(distanceFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            csv_data = [row for i,row in enumerate(csv_reader) if i>=2]

        distance  = np.zeros((grid_size**2,))
        for i,row in enumerate(csv_data):
            distance[i] = row[4]

        return M, N, distance, config, reachable, location


    def checkCollision(self, armsGridPos):
        x,y = armsGridPos[0]
        idx = x + y*self.M 
        x,y = armsGridPos[1]
        idx *= self.M*self.N
        idx = x + y*self.M 

        if self.distance[idx] <= self.min_dist:
            return True

        return False

    #TODO: esto lo he copiado de env_pickplace..mmm dejalo como deberia
    def _xy2idx(self, xy):
        ''' xy grid pos to index '''
        x,y = xy
        return x + y*self.M

    def checkValidLocation(self, armsGridPos):
        for pos, reach in zip(armsGridPos, self.reachable):
            idx = self._xy2idx(pos)
            if not reach[idx]:
                return False

        return True


    def getConfig(self):
        return self.config


    def updateLocation(self, idx, value):
        self.location[idx] = value


    def plot(self, phyPos, ax, plt3d):
        ''' Plot robot arms. It only shows the EE position '''
        x,y,z = [],[],[]
        for xi,yi,zi in phyPos:
            x.append(xi)
            y.append(yi)
            z.append(0)

        if plt3d: ax.scatter3D(x, y, z, c='k', marker ='o', s=44)
        else:     ax.scatter  (x, y,    c='k', marker ='o', s=44)

        ax.plot([-200, phyPos[0][0]], [100, phyPos[0][1]], 'k:')
        ax.plot([ 200, phyPos[1][0]], [100, phyPos[1][1]], 'k:')
