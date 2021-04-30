
import numpy as np
import tinyik                                      # direct and inverse kinematic
import lines3ddistance as l2l                      # distance between 2 segment lines in 3D


class Robot:

    def __init__(self, cfg, work_area):
        ''' Init robot '''
        self.cfg = cfg
        self.robot = [ tinyik.Actuator(arm_cfg['model']) for arm_cfg in cfg ]

        x, y    =  work_area['rect'][0],  work_area['rect'][1]  # top-left 2D grid corner
        M = work_area['size'][0]
        N = work_area['size'][1]
        x_delta = (work_area['rect'][2] - work_area['rect'][0])/(N-1)
        y_delta = (work_area['rect'][1] - work_area['rect'][3])/(M-1)
        self.grid = [ [x + x_delta*j, y + y_delta*k, 0] for j in range(N) for k in range(M)]

        self.reset()
        self.M = M
        self.N = N

    def reset(self):
        ''' Set initial robot pose '''
        self.setAngles([arm_cfg['defaultJoints'] for arm_cfg in self.cfg])

    def setAngles(self, angles):
        ''' Update robot angles '''
        for i, arm_angles in enumerate(angles):
            self.robot[i].angles = arm_angles

    def getAngles(self):
        ''' Read robot angles '''
        return [arm.angles for arm in self.robot]

    def setEE(self, ee):
        ''' Update robot end-effector position (global coordinates system) '''
        for i, arm_ee in enumerate(ee):
            self.robot[i].ee = np.array(arm_ee) - self.cfg[i]['modelPos'] # global (env) to local (robot) coord system

    def getEE(self):
        ''' Read robot end-effector positions (global coordinates system) '''
        return [arm.ee + cfg['modelPos'] for arm, cfg in zip(self.robot, self.cfg)] # local (robot) to global (env) coord system

    def getLinksPos(self):
        ''' Read links pos
            It is useful to plot the robot model
        '''
        linksPos = [[np.array(cfg['modelPos'])] for cfg in self.cfg]
        for i, arm in enumerate(self.robot):
            for j in range(len(arm.angles)):
                linksPos[i].append(arm._fk.solve_n(arm.angles, 2**(j+1))[0:3] + self.cfg[i]['modelPos'])

        return linksPos

    def getNumArms(self):
        ''' Get number of arms '''
        return len(self.robot)

    def checkReachability(self, armsGridPos):
        pass

    def checkCollision(self, pose, margin=0.1):
        ''' Check robot collision in its current pose

            Margin argument defines the minimum distance allowed between arms. 
            (lower than margin distance is considered collision)
        '''

        self.setAngles(pose)
        numArms = len(self.robot)
        links = self.getLinksPos()
        for i in range(numArms):
            for j in range(i+1, numArms):
                # Check collision on any pair of arms
                link1_b = links[i][0]
                for link1 in links[i][1:]:
                    link1_a = link1_b
                    link1_b = link1

                    link2_b = links[j][0]
                    for link2 in links[j][1:]:
                        link2_a = link2_b
                        link2_b = link2
                        _, _, dist = l2l.closestDistanceBetweenLines(link1_a, link1_b, link2_a, link2_b, clampAll=True)
                        if dist <= margin:
                            # At least two links are colliding
                            return True

        return False

    def checkPathCollision(self, poseA, poseB):
        ''' Check robot collision when moving from current pose to a new one 
        
            Trajectory is discretized in N steps and collision check is 
            performed on all (N) intermediate poses 
        '''
        ## robot angles for current and target pos
        robot_ang_0 = poseA
        robot_ang_1 = poseB

        # Angular increment for each joint
        path_step = 4
        robot_diff = []
        for arm_ang_0, arm_ang_1 in zip(robot_ang_0, robot_ang_1): # per arm
            arm_diff = []
            for ang_0, ang_1 in zip(arm_ang_0, arm_ang_1):         # per joint
                tmp = (360 + np.degrees(ang_1) - np.degrees(ang_0)) % 360
                arm_diff.append(np.radians(tmp if tmp<180 else tmp-360))
            robot_diff.append(np.array(arm_diff)/path_step) 

        # Build a list of intermediary poses (from current pos to target pos)
        poses = []
        for i in range(path_step):
            poses.append(np.array(robot_ang_0) + np.array(robot_diff)*(i+1))

        # Check collision during path
        collision = False
        for pose in poses:
            if self.checkCollision(pose, margin=0.3):
                collision = True
                break

        return collision

    def getGridAngles():
        ''' Compute robot (angles) configurations 

            Trick: Set an initial robot configuration and then move across the
                   the grid (by little steps), reading the robot configuration.
        '''

        self.setAngles([[np.pi, 0.3], [0, -1]]) # Initial position (very near to first position)
        
        nArms = self.getNumArms()
        self.gridAng = [ [] for _ in range(nArms)]
        for pos in self.grid:
            # Re-position robot and read configuration
            self.setEE([pos for _ in range(nArms)])
            angles = self.getAngles()
            for i,ang in enumerate(angles):
                self.gridAng[i].append(ang)

    def updateGrid(lpos, ppos):
        ''' Update #grid positions where pieces are assigned '''
        #idx = ...(lpos)
        #self.grid[idx] = ppos
        pass

    def targetGetTime(self, target):
        ''' Measure time (s) from current pose to a new one

            'target' is a list of points in 3D space (one per arm)

            Note: all joints move simultanenously (start/stop at the same time)
        '''

        # TODO: take into account the joint speed
        sec = 0
        for pos, arm, cfg in zip(target, self.robot, self.cfg):
            a0 = arm.angles
            arm.setEE = pos
            a1 = arm.angles

            # Focus on the slowest joint (angle increment/angular velocity)
            tmp  = (360 + np.degrees(a1) - np.degrees(a0)) % 360
            tmp[tmp>180] = 360 - tmp[tmp>180]
            sec = max(sec, max(tmp/cfg['jointsSpeed']))

        # Assume constant angular velocity
        return sec 


    def plot(self, armsGridPos, ax, plt3d):
        ''' Draw EEs position (just a circle) '''
        for vertices in self.getLinksPos():
            x,y,z =  np.array(vertices).T

            if plt3d:
                ax.plot3D(x, y, z, c='grey', lw=8)
                ax.plot3D(x, y, z, c='r', marker='o')
            else:
                ax.plot(x, y, c='grey', lw=8)
                ax.plot(x, y, c='r', marker='o')
