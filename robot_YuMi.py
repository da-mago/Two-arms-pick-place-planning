
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

    def __init__(self, globalCfg):
        ''' Define the robot model and create an instance of Robot class '''

        self.min_distance = globalCfg.collision_min_distance # Distance considered collision (cm)
        self.z_layers     = globalCfg.grid_num_layers
        self.z            = 180   # Default z value for robot EEs
        self.unknown_pos  = [-1,-1,-1]

        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance.csv')    
        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'AlcanceWo200M150180.csv')    
        #self.M, self.N, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance_v5.csv')    
        #self.M, self.N, self.Z, self.distance, self.config, self.reachable, self.location = self._loadCSV('Collision_v3.csv', 'Alcance_v11.csv')    
        #self.M, self.N, self.Z, self.distance, self.config, self.reachable, self.location = self._loadCSV('Colision_grid3D_v5.csv', 'Alcance_grid3D_v4.csv')    
        self.M, self.N, self.Z, self.distance, self.config, self.reachable, self.location = self._loadCSV('Colision_grid3D_v6.csv', 'Alcance_grid3D_v4.csv')    

    def _loadCSV(self, distanceFilename, configFilename):
        ''' Read and process robot data from csv file '''

        # Read external data
        #
        # 1. Robot configuration
        with open(configFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            #csv_data = [row for i,row in enumerate(csv_reader) if i>=3]
            csv_data = [row for row in csv_reader]

        # Read metadata (grid size)
        M  = int(csv_data[0][1])
        N  = int(csv_data[0][2])
        Z  = int(csv_data[0][3])
        Z = self.z_layers
        NUM_ARMS = 2
        NUM_ANGLES = 7
        csv_data = csv_data[3:] # Skip non-arm-data rows

        location  = np.zeros((M, N, Z, 3), dtype=np.int16)
        reachable = np.zeros((NUM_ARMS, M, N, Z), dtype=np.uint8)
        config    = np.zeros((NUM_ARMS, M, N, Z+1, NUM_ANGLES), dtype=np.float) #+1 for open/close poses
        idx = 0
        for z in range(Z+1):
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

                    # Excel format (first, open/close plane (z=0), then grid 3D)
                    cfg_l = 4  # index in the line (comma separated)
                    cfg_r = 15 
                    reach_l = 3
                    reach_r = 14
                    if z == 0:
                        config[0, x, y, Z] = [float(t) for t in row[cfg_r : cfg_r + NUM_ANGLES]]  # left
                        config[1, x, y, Z] = [float(t) for t in row[cfg_l : cfg_l + NUM_ANGLES]]  # right

                    else:
                        # Physical EE location
                        location[x, y, z-1] = [-int(row[1]), int(row[0]), int(row[2])]
                        # Reachable EE location
                        reachable[0, x, y, z-1] = int(row[reach_r]) # left
                        reachable[1, x, y, z-1] = int(row[reach_l]) # right
                        # Pose (angles)
                        config[0, x, y, z-1] = [float(t) for t in row[cfg_r : cfg_r + NUM_ANGLES]]  # left
                        config[1, x, y, z-1] = [float(t) for t in row[cfg_l : cfg_l + NUM_ANGLES]]  # right


        # 2. Robot collision (distance between arms)
        with open(distanceFilename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            csv_data = [row for i,row in enumerate(csv_reader) if i>=1]

        distance  = np.zeros((M, N, Z, M, N, Z), dtype=np.uint16)
        idx = 0
        # Loop over excel file (order given by the excel format itself)
        excelZ = 3 # Trick to enable grid 3D with only 2 layers (being 3 layers in excel)
        for zl in range(excelZ):
            for zr in range(excelZ):
                for yl in range(N):
                    for xl in range(M-1, -1, -1):
                        for yr in range(N):
                            for xr in range(M-1, -1, -1):
        
                                # Again... reorganize csv data
                                row = csv_data[idx]
                                idx += 1

                                if zl >= Z or zr >= Z:
                                    continue

                                # Consider any value below 29mm given by RobotStudio as collision
                                # For non collision configurations, take the distance between EEs.
                                #robotStudio_distance = int(row[6]) * int(float(row[7]))
                                #if robotStudio_distance < 29:
                                ##robotStudio_distance = int(row[6]) 
                                ##if robotStudio_distance == 0:
                                #    dist = 0
                                #else:
                                #    dist = int(np.sqrt( (100*(xl-xr))**2 + (100*(yl-yr))**2 + (100*(zl-zr))**2 )) # units: mm

                                dist = int(float(row[6]))
                                distance[xl,yl,zl,xr,yr,zr] = dist 

        return M, N, Z, distance, config, reachable, location

    def checkCrossCollision(self, preArmsGridPos, postArmsGridPos):
        ''' Check potential collision during the move '''

        # Ignore the check if any arm pos is unknown
        if self.unknown_pos in preArmsGridPos or \
           self.unknown_pos in postArmsGridPos:
            assert False, 'OK... not sure if this happens'
            return False

        xl,yl,zl = (np.array(preArmsGridPos[0]) + np.array(postArmsGridPos[0]))/2
        xr,yr,zr = (np.array(preArmsGridPos[1]) + np.array(postArmsGridPos[1]))/2
        dist = int(np.sqrt( (100*(xl-xr))**2 + (100*(yl-yr))**2 + (100*(zl-zr))**2 )) # units: 

        return False
        return (dist < 1) # Collision if distance is less than one cell

    def checkCollision(self, armsGridPos):
        # Ignore the check if any arm pos is unknown
        if self.unknown_pos in armsGridPos:
            #assert False, 'OK... not sure if this happens'
            return False

        (xl,yl,zl),(xr,yr,zr) = armsGridPos

        return (self.distance[xl,yl,zl,xr,yr,zr] < self.min_distance)

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
            if pos != self.unknown_pos:
                x,y,z = pos
                if not reach[x,y,z]:
                    return False
            else:
                # Ignore the check if the arm pos is unknown
                pass
                #assert False, 'OK... not sure if this happens in checkValidLocation'

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
