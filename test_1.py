
from mdp_generator import mdp_generator
from mdp_solver import mdp_solver
from pick_place import generateTxtPlan
from robot_YuMi import Robot_YuMi


robot = Robot_YuMi()

#                    Piece 1              Piece 2             Piece 3            Piece 4
#              pick      place
test_cases = [ [ [300, 350, 500,-150], [600, 150,300,-150],[500, 250,200,-350],[300,250,500,-250]],
               [ [500, 250, 200,-350], [300, 350,500,-150],[300, 150,600,-150],[300,250,500,-250]],
               [ [200,-250, 500, -50], [200,-350,500, 150],[300, 350,600, -50],[200,250,600,-150]],
               [ [300,-350, 500, 150], [500,-250,600, -50],[400, 150,400,-250],[300,150,200,-350]],
               [ [500,-250, 200, 350], [500, 150,300, -50],[300,-350,600, -50],[400,-50,600, 150]] ]

for i,test in enumerate(test_cases):

    pieces = [{'start': [-t[1], t[0], 0], 'end': [-t[3], t[2], 0]} for t in test]

    robot_mdp = mdp_generator(robot, pieces)
    robot_mdp.update()
    state_fnc, reward_fnc = robot_mdp.MDP[0], robot_mdp.MDP[1] 
    solver = mdp_solver([robot_mdp.MDP[0], robot_mdp.MDP[1]])
    policy = solver.solve()
    
    armsGridPos = [[4, 3], [7, 4]]
    src = generateTxtPlan(policy, armsGridPos, pieces, robot_mdp)
    
    with open("prueba_00{}.txt".format(i+1), "w") as f:
        f.write(src)


