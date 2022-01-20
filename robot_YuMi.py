
#from robot import Robot
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

    def __init__(self, min_dist=50, z=180):
        ''' Define the robot model and create an instance of Robot class '''

        self.min_dist = min_dist # Distance considered collision (cm)
        self.z = z               # Default z value for robot EEs

        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance.csv')    
        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'AlcanceWo200M150180.csv')    
        self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance_v4.csv')    


    def _loadCSV(self, distanceFilename, configFilename):
        ''' Read and process robot data from csv file '''

        # Read external data
        #
        # 1. Robot configuration
        with open(configFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            #csv_data = [row for i,row in enumerate(csv_reader) if i>=3]
            csv_data = [row for i,row in enumerate(csv_reader)]

        #grid_size = len(csv_data)
        M  = int(csv_data[0][1])
        N  = int(csv_data[0][2])
        Zs = int(csv_data[0][4])
        grid_size = M*N
        angles_size = 7
        csv_data = csv_data[3:] # Skip non-arm-data rows

        location  = np.zeros((grid_size, 3), dtype=np.int16)
        reachable = np.zeros((2, grid_size), dtype=np.uint8)
        #config    = np.zeros((2, grid_size, angles_size), dtype=np.float) 
        config    = np.zeros((2, Zs, grid_size, angles_size), dtype=np.float) 
        #M = len(set([ int(row[1]) for row in csv_data]))
        #N = len(set([ int(row[0]) for row in csv_data]))
        idx = 0
        for y in range(N):
            for x in range(M):
                # Let's reorganize csv data
                #
                #      x      y
                #      ^      ^
                #      |      |
                #  y<--o      o-->x
                #  csv        Robot
                #
                i = (M-1-x) + y*M
                row = csv_data[idx]
                idx += 1

                location[i] = [-int(row[1]), int(row[0]), self.z]
                #reachable[0, i] = 1 if row[10]=='Si' else 0           # valid EE location (left)
                #reachable[1, i] = 1 if row[2 ]=='Si' else 0           # right
                reachable[0, i] = int(row[2+10])                       # valid EE location (left)
                reachable[1, i] = int(row[2 ])                         # right
                config[0, 0, i] = [float(t) for t in row[3+10:10+10]]  # 7 angles (left)
                config[1, 0, i] = [float(t) for t in row[3:10]]        # right

        # Get data for other Zs grids (used when moving up/down)
        for otherZ in range(Zs - 1):
            idx += 2 # there are 2 comment lines between blocks
            for y in range(N):
                for x in range(M):
                    # Read Reachable and config for other Zs (used in pick/drop operations)
                    i  = (M-1-x) + y*M
                    row = csv_data[idx]
                    idx += 1

                    #reachable[0, i] = int(row[2+10])                    # valid EE location (left)
                    #reachable[1, i] = int(row[2 ])                      # right
                    config[0, otherZ+1, i] = [float(t) for t in row[3+10:10+10]]  # 7 angles (left)
                    config[1, otherZ+1, i] = [float(t) for t in row[3:10]]        # right

        # 2. Robot collision (distance between arms)
        with open(distanceFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            csv_data = [row for i,row in enumerate(csv_reader) if i>=1]

        distance  = np.zeros((grid_size**2,), dtype=np.uint16)
        idx = 0
        for yl in range(N):
            for xl in range(M):
                for yr in range(N):
                    for xr in range(M):
                        # Again... reorganize csv data
                        row = csv_data[idx]
                        idx += 1
                        # 'M-1-x' (inverse x) and lowest range for 'xr-yr' according to the excel data organization
                        i = ((M-1-xl) + yl*M)*M*N + (M-1-xr) + yr*M

                        # Consider any value below 28mm given by RobotStudio as collision
                        # For non collision configurations, take the distance between EEs.
                        robotStudio_distance = int(row[7])*int(float(row[9]))
                        #if robotStudio_distance == 0:
                        if robotStudio_distance < 29:
                            distance[i] = 0
                        else:
                            # Last hour change (to mm)
                            #distance[i] = int(np.sqrt( (10*(xl-xr))**2 + (10*(yl-yr))**2 )) # units: cm
                            distance[i] = int(np.sqrt( (100*(xl-xr))**2 + (100*(yl-yr))**2 )) # units: cm
                            #if i == 2491:
                            #    print(i, xl,yl,xr,yr, distance[i], int(np.sqrt( (100*(xl-xr))**2 + (100*(yl-yr))**2 )) )
                            #distance[i] = 100


        return M, N, distance, config, reachable, location


    def checkCollision(self, armsGridPos):
        # Ignore the check if any arm pos is unknown
        if [-1,-1] in armsGridPos:
            return False

        idx = self._xyxy2idx(armsGridPos)
        if self.distance[idx] <= self.min_dist:
            return True

        return False

    #TODO: esto lo he copiado de env_pickplace..mmm dejalo como deberia
    def _xyxy2idx(self, xyxy):
        ''' xyxy grid pos to index '''
        (xl,yl),(xr,yr) = xyxy
        return (xl + yl*self.M)*self.M*self.N + xr + yr*self.M
        #return ((self.M-1-xl) + yl*self.M)*self.M*self.N + (self.M-1-xr) + yr*self.M

    def _xy2idx(self, xy):
        ''' xy grid pos to index '''
        x,y = xy
        return x + y*self.M

    def checkValidLocation(self, armsGridPos):
        for pos, reach in zip(armsGridPos, self.reachable):
            if pos != [-1,-1]: # Ignore the check if the arm pos is unknown
                idx = self._xy2idx(pos)
                if not reach[idx]:
                    return False

        ## Trampa para que el brazo izquierdo no pase de cierta posicion en el eje X. Es algo temporal para generar una solucion trucada para llevarla a RobotStudio
        ## Si y<=400 and x>=250, o y>=500 and x>=150
        #x,y = armsGridPos[0][0], armsGridPos[0][1]
        #if y <= 2:
        #    if x >= 7:
        #        return False
        #else:
        #    if x >= 6:
        #        return False

        #print()
        #print("check")
        #print(armsGridPos)


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

        #  Robot      Plot
        # 
        #      x      y
        #      ^      ^
        #      |      |
        #  y<--o      o-->x
        if plt3d: ax.scatter3D(x, y, z, c='k', marker ='o', s=44)
        else:     ax.scatter  (x, y,    c='k', marker ='o', s=44)

        ax.plot([-150, phyPos[0][0]], [100, phyPos[0][1]], 'k:') # left (virtual dashed) arm
        ax.plot([ 150, phyPos[1][0]], [100, phyPos[1][1]], 'k:')
