
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
        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance_v5.csv')    
        self.M, self.N, self.Z, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance_v11.csv')    

    def _loadCSV(self, distanceFilename, configFilename):
        ''' Read and process robot data from csv file '''

        # Read external data
        #
        # 1. Robot configuration
        with open(configFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            #csv_data = [row for i,row in enumerate(csv_reader) if i>=3]
            csv_data = [row for i,row in enumerate(csv_reader)]

        M  = int(csv_data[0][1])
        N  = int(csv_data[0][2])
        Z  = 3 # pending to new excel format for 3D grid
        Zs = int(csv_data[0][4])
        NUM_ARMS = 2
        ANGLES_SIZE = 7
        csv_data = csv_data[3:] # Skip non-arm-data rows

        location  = np.zeros((M, N, Z, 3), dtype=np.int16)
        reachable = np.zeros((NUM_ARMS, M, N, Z), dtype=np.uint8)
        config    = np.zeros((NUM_ARMS, M, N, Z+1, ANGLES_SIZE), dtype=np.float) #+1 for open/close poses
        idx = 0
        for y in range(N):
            for x in range(M-1, -1, -1):
                # Let's reorganize csv data
                #
                #      x      y
                #      ^      ^
                #      |      |
                #  y<--o      o-->x
                #  csv        Robot
                #
                row = csv_data[idx]
                idx += 1

                # TODO: update excel format for 3D grid
                # Physical EE location
                location[x, y, :] = [-int(row[1]), int(row[0]), 180]
                # Reachable EE location
                reachable[0, x, y, :] = int(row[2+10])                    # left
                reachable[1, x, y, :] = int(row[2])                       # right
                # Pose (angles)
                config[0, x, y, :] = [float(t) for t in row[3+10:10+10]]  # left
                config[1, x, y, :] = [float(t) for t in row[3:10]]        # right

        # Get data for open/close position
        idx += 2 # there are 2 comment lines between blocks
        for y in range(N):
            for x in range(M-1, -1, -1):
                # Read Reachable and config for other Zs (used in pick/drop operations)
                row = csv_data[idx]
                idx += 1

                config[0, x, y, Z] = [float(t) for t in row[3+10:10+10]]  # 7 angles (left)
                config[1, x, y, Z] = [float(t) for t in row[3:10]]        # right

        # 2. Robot collision (distance between arms)
        with open(distanceFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            csv_data = [row for i,row in enumerate(csv_reader) if i>=1]

        distance  = np.zeros((M, N, Z, M, N, Z), dtype=np.uint16)
        idx = 0
        for yl in range(N):
            for xl in range(M-1, -1, -1):
                for yr in range(N):
                    for xr in range(M-1, -1, -1):
                        zl = zr = 0
                        # Again... reorganize csv data
                        row = csv_data[idx]
                        idx += 1

                        # Consider any value below 29mm given by RobotStudio as collision
                        # For non collision configurations, take the distance between EEs.
                        robotStudio_distance = int(row[7]) * int(float(row[9]))
                        if robotStudio_distance < 29:
                            dist = 0
                        else:
                            dist = int(np.sqrt( (100*(xl-xr))**2 + (100*(yl-yr))**2 + (100*(zl-zr))**2 )) # units: mm

                        # TODO: update excel format for 3D grid
                        distance[xl,yl,0,xr,yr,0] = dist 
                        if Z > 1:
                            distance[xl,yl,1,xr,yr,1] = dist 
                        if Z > 2:
                            distance[xl,yl,2,xr,yr,2] = dist 
                        if Z > 1:
                            distance[xl,yl,0,xr,yr,1] = 100 # By now, there is no collision between Zs 
                            distance[xl,yl,1,xr,yr,0] = 100 
                        if Z > 2:
                            distance[xl,yl,0,xr,yr,2] = 100 
                            distance[xl,yl,1,xr,yr,2] = 100 
                            distance[xl,yl,2,xr,yr,0] = 100 
                            distance[xl,yl,2,xr,yr,1] = 100 

        return M, N, Z, distance, config, reachable, location


    def checkCollision(self, armsGridPos):
        # Ignore the check if any arm pos is unknown
        if [-1,-1,-1] in armsGridPos:
            assert False, 'OK... not sure if this happens'
            return False

        (xl,yl,zl),(xr,yr,zr) = armsGridPos

        return (self.distance[xl,yl,zl,xr,yr,zr] <= self.min_dist)

    #TODO: esto lo he copiado de env_pickplace..mmm dejalo como deberia
    def _armsPos2idx(self, armsPos):
        ''' Grid pos (both arms) to scalar '''
        res = 0
        for x,y,z in armsPos:
            res *= self.M*self.N*self.Z
            res += x + y*self.M + z*self.M*self.N
        return res

    def _xy2idx(self, singleArmPos):
        ''' Grid pos (single arm) to scalar '''
        x,y,z = singleArmPos
        return x + y*self.M + z*self.M*self.N

    def checkValidLocation(self, armsGridPos):
        for pos, reach in zip(armsGridPos, self.reachable):
            if pos != [-1,-1,-1]: # Ignore the check if the arm pos is unknown
                x,y,z = pos
                if not reach[x,y,z]:
                    return False
            else:
                assert False, 'OK... not sure if this happens in checkValidLocation'


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


    def updateLocation(self, armPos, value):
        x,y,z = armPos
        self.location[x,y,z] = value


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
