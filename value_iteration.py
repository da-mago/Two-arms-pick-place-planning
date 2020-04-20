# TODO:
# - ADD intermediate states for pick&place actions
# - refactor code for collision/reach info in excel
# - pieces pos sharing the same grid point
# - try 20x10 grid: state space is 16 times bigger -> IDEM for computation time?
# - Autopreguntas:
#   - Si mientras calcula movemos brazo hacia piezas? no creo que ganemos mucho
#   - La simulacion deberia terminar en una posicion de reposo?
#   - Si ponemos una posicion de reposo cerca de las piezas?
# Book: planning with Markov decission processes

import numpy as np
import sys
import pickle
import envYumi
import time    # sleep
import imageio # mimsave

def FasterValueIteration(env, V, states7, theta=0.0001, discount_factor=1.0):
    ''' Avoid python code :). Let's numpy do the magic.
        Do not iterate for each state, make all states computations at once
        at python level
    '''

    while True:
        #V_cp = np.copy(V)
        #V1 = np.max( env.MDP[1] + discount_factor*V_cp[env.MDP[0]], axis=1 )
        #delta = np.max(np.abs(V-V_cp))
        Vk = np.max( env.MDP[1] + discount_factor*V[env.MDP[0]], axis=1 )
        delta = np.max(np.abs(Vk-V))
        V = Vk
        if delta < theta:
            break

    ## Extra performance: compute Value F on smaller blocks
    ## TODO: not really optimal (solution with more steps). Need to be review the blocks created
    #for states in p_states_val:
    #    if len(states) == 0:
    #        continue
    #    while True:
    #        V_cp = np.copy(V)
    #        V[states] = np.max( env.MDP[1][states] + discount_factor*V_cp[env.MDP[0][states]], axis=1 )
    #        delta = np.max(np.abs(V[states]-V_cp[states]))
    #        if delta < theta:
    #            break

    policy = ComputeOptimalPolicies(V)

    return V, policy

def ValueIteration(env, V, states, theta=0.0001, discount_factor=1.0):

    for states in p_states_val:
        ComputeOptimalValueF(env, V, states, theta=0.0001, discount_factor=1.0)
    policy = ComputeOptimalPolicies(V)

    return V, policy


def ComputeOptimalValueF(env, V, states, theta=0.0001, discount_factor=1.0):
    """
    Value Iteration Algorithm.
    
    Args:
        env: OpenAI env. 
            env.MDP encodes (as a table) transition and reward funtions
            env.MDP[s][a] is a list of transition tuples (next_state, reward, valid_state).
            env.nS is the number of available states
            env.nA is the number of available actions
        theta: We stop evaluation once our value function change is less than theta for all states.
        discount_factor: Gamma discount factor.
        
    Returns:
        A tuple (policy, V) of the optimal policy and the optimal value function.
    """

    if len(states) == 0:
        return V

    done = False
    cnt = 0
    while not done:
        # Stopping condition
        updated = 0 
        delta = 0
        done = True
        V_cp = np.copy(V)
        Vt = []

        # For every valid state
        for state in states:

            # Update the value function
            idx = states_idx[state]
            #next_states_idx = np.array( [states_idx[next_state] for next_state in env.MDP[0][idx]], dtype=np.int32 )
            next_states_idx = env.MDP[0][idx]
            # V[s] =       reward        +         gamma    V[s']
            V[idx] = np.max( env.MDP[1][idx] + discount_factor*V_cp[next_states_idx] )
            Vt.append(V[idx])

            # Calculate delta across all states seen so far
            curr_delta = np.abs(V[idx] - V_cp[idx])
            if curr_delta > theta:
                done = False
                updated += 1 # debug counter (states changing more than theta)
                #print(i, env.MDP[2][i], curr_delta)
            delta = max(delta, curr_delta)
            #if i%1000 == 0:
            #    print(i)

        cnt += 1
        if cnt>33:
            break
        #print(cnt, updated)
        #print(cnt, updated, delta)
        
    #print('Done')

    return V
    

def ComputeOptimalPolicies(V, discount_factor = 1.0):
    ''' Compute optimal policies from Optimal Value Function '''

    #policy = np.zeros(states_len, dtype=np.int8)
    #for i in range(states_len):
    #    policy[i] = np.argmax(env.MDP[1][i] + discount_factor*V[env.MDP[0][i]])
    policy = np.array([np.argmax(env.MDP[1][i] + discount_factor*V[env.MDP[0][i]]) for i in range(states_len)])
    
    return policy

def MDPCompletePickplace():
    ''' MDP table is generated in two phases:
        - Offline phase: compute the MDP assuming that there are no pieces (so 
                         pick&place actions are always invalid). 
        - Online phase:  update the MDP for a specific configuration of the
                         pieces
        The point is to minimize the online (real time) computations.
    '''

    offset_a = np.array([[1,0],[-1,0],[0,1],[0,-1]]) # rigth, left, down, up

    p_ini = [cfg['start'] for cfg in env.piecesGridCfg]
    p_end = [cfg['end']   for cfg in env.piecesGridCfg]

    for arm in range(2):
        for i, (pos_ini,pos_end) in enumerate(zip(p_ini, p_end)):
            for x in range(env.M):
                for y in range(env.N):
                    for status_2 in range(env.K+1):
                        for bitmap in range(2**(env.K)):
                            if bitmap & (1<<i):
                                # pick
                                pos = pos_ini
                                status = 0
                                next_status = i+1
                                tmp_bitmap = bitmap & ~(1<<i)
                                a = 4
                                # place
                            else:
                                pos = pos_end
                                status = i+1
                                next_status = 0
                                tmp_bitmap = bitmap
                                a = 5

                            if arm == 0:
                                state = env._int2extState([pos, [x,y]], [status, status_2], bitmap)
                            else:
                                state = env._int2extState([[x,y], pos], [status_2, status], bitmap)

                            # Process only valid states
                            if state in states_idx:
                                
                                idx = states_idx[state]
                                # Avoid checks (it was done in phase 1, -30 means all checks were passed)
                                for action in range(7):
                                    if arm == 0: a1 = a;      a2 = action
                                    else:        a1 = action; a2 = a

                                    next_bitmap = tmp_bitmap
                                    if env.MDP[1][idx][a1*7+a2] == -30: # pick/any
                                        # remove mark
                                        env.MDP[1][idx][a1*7+a2] = -20

                                        next_status_2 = status_2
                                        if action < 4: 
                                            pos2 = list([x,y] + offset_a[action])
                                        else:
                                            pos2 = [x,y]
                                            if action == 4:
                                                if (status_2 == 0) and ([x,y] in p_ini):
                                                    tmp = p_ini.index([x,y])
                                                    if bitmap & (1 << tmp):
                                                        next_status_2 = tmp+1
                                                        next_bitmap &= ~(1<<tmp)
                                                    else:
                                                        continue
                                                else:
                                                    continue
                                            elif action == 5:
                                                if (status_2 > 0) and ([x,y] in p_end):
                                                    tmp = p_end.index([x,y])
                                                    if (bitmap & (1 << tmp)) == 0 and status_2 == tmp+1:
                                                        next_status_2 = 0
                                                    else:
                                                        continue
                                                else:
                                                    continue

                                        # update next_state and reward
                                        if arm == 0: state =  env._int2extState([pos,pos2], [next_status,next_status_2], next_bitmap)
                                        else:        state =  env._int2extState([pos2,pos], [next_status_2,next_status], next_bitmap)

                                        if next_bitmap + np.sum([next_status, next_status_2]) == 0: reward = 100
                                        elif action == 6:                                           reward = -2
                                        else:                                                       reward = -3

                                        env.MDP[0][idx][a1*7+a2] = states_idx[state]
                                        env.MDP[1][idx][a1*7+a2] = reward


if __name__ == "__main__":

    env = envYumi.EnvYuMi()
    env.reset()

    states_val = env.MDP[2]
    states_len = len(states_val)
    states_idx = {}
    for i, item in enumerate(states_val):
        states_idx[item] = i

    #MDP2 = pickle.load( open( "MDP_complete.bin", "rb" ) )

    # Complete MDP (pick/place actions)
    MDPCompletePickplace()

    #sys.exit()

    #cnt=0
    #for i in range(len(MDP2[0])):
    #  for j in range(len(MDP2[0][0])):
    #    if MDP2[0][i][j] != env.MDP[0][i][j] or MDP2[1][i][j] != env.MDP[1][i][j]:
    #      if MDP2[1][i][j] !=  -20:
    #          print('failure', i, env._ext2intAction(j), env._ext2intState(MDP2[2][i]),  env._ext2intState(MDP2[0][i][j]), env._ext2intState(env.MDP[0][i][j]))
    #          print(MDP2[1][i][j], env.MDP[1][i][j])
    #          cnt+=1
    #print(cnt)

    #sys.exit()

    #env.MDP[0] = np.array([[states_idx[env.MDP[0][i][j]] for j in range(len(env.MDP[0][0]))] for i in range(states_len)])

    #sys.exit()

    ## This piece of code is only meaningful when computing Value F in small blocks
    #p_states_val = [[] for _ in range(16*24)]
    #levels = [0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0x7, 0xb, 0xd, 0xe, 0xf]
    #for i, state in enumerate(states_val):
    #    armsGridPos, armsStatus, piecesMap = env._ext2intState(state)
    #    idx = levels[piecesMap]
    #    idx *= 24
    #    if sum(armsStatus) == 0:
    #        pass
    #    elif armsStatus[0] == 0:
    #        idx += armsStatus[1]
    #    elif armsStatus[1] == 0:
    #        idx += 4+armsStatus[0]
    #    else:
    #        idx += 8+1+(armsStatus[0]-1)*4+(armsStatus[1]-1)

    #    #p_states_val[idx].append(state)
    #    #p_states_val[idx].append(i)

    #for i,item in enumerate(p_states_val):
    #    print(i, len(item))

    #p_states_val = [states_val[i*states_len//4:(i+1)*states_len//4] for i in range(4)]
    p_states_val = [states_val] # Prioritized states blocks

    V = np.zeros(states_len, dtype=np.float)
    #V = np.zeros(states_len, dtype=np.int8)
    #sys.exit()
    print(states_len)

    # Train
    if True:
        #V = pickle.load( open( "value.bin", "rb" ) )

        # Compute Value Function in separated and smaller blocks instead of all at once
        #
        # Reason: goal reward is propagated from the last state to all the rest. When 
        #         computing it at once, some states will converge before others, some 
        #         states will be updated with meaningful info until several iterations, ...
        #         If we are smart, we can take advantadge of this knowledge (do not train
        #         Value Funtion in states representing all pieces are not picked up yet,
        #         if the others states representing more avanced positions in the game have
        #         not converge yet)
        #
        #V, policy = ValueIteration(env, V, p_states_val, discount_factor=0.95)
        V, policy = FasterValueIteration(env, V, p_states_val, discount_factor=0.95)

        #pickle.dump( v,      open( "value.bin", "wb" ) )
        #pickle.dump( policy, open( "policy.bin", "wb" ) )
    else:
        policy = pickle.load( open( "policy.bin", "rb" ) )

    #sys.exit()
    # Show solution
    #valid_states = list(env.MDP[2])

    next_state = env._int2extState([[0,4],[16,4]],[0,0],15)
    env.piecesGridPos = [ cfg['start']  for cfg in env.piecesGridCfg ] # esto deberia estar en env.reset
    env.reset(next_state)
    env.render()
    done = False
    images = []
    i=0
    while done == False:
        action = policy[ states_idx[next_state] ]
        #action = policy[ valid_states.index(next_state) ]
        #action = policy[next_state]
        print(i, env._ext2intAction(action))
        i+=1
        next_state, reward, done, info = env.step(action)
        env.render()
        time.sleep(0.1)
        #print(env.armsGridPos, env.armsStatus, env.piecesMap)
        image = np.frombuffer(env.fig.canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(env.fig.canvas.get_width_height()[::-1] + (3,)) # Example: (640,480,3)
        images.append(image)
    
    imageio.mimsave('YuMi.gif', images, duration=0.2)

