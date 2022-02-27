
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

# Let's solve all two-piece subproblems and then take the optimal four-piece
# solution based on the previous ones.
n_pieces = len(test_cases[0])
for i,test in enumerate(test_cases):

    best_steps = 1000
    best_src = ''
    # For every (6) two-piece combination
    for j in range(n_pieces):
        for k in range(j+1,n_pieces):
           # First and second two-piece subproblem
           stage1 = [j,k]
           stage2 = [n for n in range(n_pieces)]
           stage2.remove(j)
           stage2.remove(k)
           print('Stage', stage1, stage2)

           armsGridPos = [[4, 3], [7, 4]]
           total_steps = 0
           total_src = ''
           for p1,p2 in [stage1, stage2]:
               pieces = [{'start': [-t[1], t[0], 0], 'end': [-t[3], t[2], 0]} for t in [test[p1], test[p2]] ]

               robot_mdp = mdp_generator(robot, pieces)
               robot_mdp.update()
               state_fnc, reward_fnc = robot_mdp.MDP[0], robot_mdp.MDP[1] 
               solver = mdp_solver([state_fnc, reward_fnc])
               policy = solver.solve()
               
               armsGridPos, num_steps, src = generateTxtPlan(policy, armsGridPos, pieces, robot_mdp)

               total_steps += num_steps
               total_src   += src

           if total_steps < best_steps:
               best_steps = total_steps
               best_src   = total_src
               
               #print(p1, p2)
               #print(num_steps)
               #print(src)
               #with open("prueba_00{}.txt".format(i+1), "w") as f:
               #    f.write(src)
    print("Total steps: {}\n{}".format(best_steps, best_src))
    with open("prueba_00{}_2x2.txt".format(i+1), "w") as f:
        f.write(best_src)


