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
        V_cp = np.copy(V)
        V = np.max( env.MDP[1] + discount_factor*V_cp[env.MDP[0]], axis=1 )
        delta = np.max(np.abs(V-V_cp))
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

    policy = np.zeros(states_len, dtype=np.int8)
    for i in range(states_len):
        #next_states_idx = np.array([states_idx[next_state] for next_state in env.MDP[0][i]], dtype=np.int32)
        next_states_idx = env.MDP[0][i]
        #print(next_states_idx)
        policy[i] = np.argmax(env.MDP[1][i] + discount_factor*V[next_states_idx])
    
    return policy


if __name__ == "__main__":

    env = envYumi.EnvYuMi()
    env.reset()

    states_val = env.MDP[2]
    states_len = len(states_val)
    states_idx = {}
    for i, item in enumerate(states_val):
        states_idx[item] = i

    #for i in range(states_len):
    #    for j in range(len(env.MDP[0][0])):
    #        env.MDP[0][i][j] = states_idx[env.MDP[0][i][j]]
    env.MDP[0] = np.array([[states_idx[env.MDP[0][i][j]] for j in range(len(env.MDP[0][0]))] for i in range(states_len)])

    p_states_val = [[] for _ in range(16*24)]
    levels = [0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0x6, 0xa, 0xc, 0x7, 0xb, 0xd, 0xe, 0xf]
    for i, state in enumerate(states_val):
        armsGridPos, armsStatus, piecesMap = env._ext2intState(state)
        idx = levels[piecesMap]
        idx *= 24
        if sum(armsStatus) == 0:
            pass
        elif armsStatus[0] == 0:
            idx += armsStatus[1]
        elif armsStatus[1] == 0:
            idx += 4+armsStatus[0]
        else:
            idx += 8+1+(armsStatus[0]-1)*4+(armsStatus[1]-1)

        #p_states_val[idx].append(state)
        p_states_val[idx].append(i)

    #for i,item in enumerate(p_states_val):
    #    print(i, len(item))

    #p_states_val = [states_val[i*states_len//4:(i+1)*states_len//4] for i in range(4)]
    #p_states_val = [states_val] # Prioritized states blocks

    V = np.zeros(states_len, dtype=np.float)
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
    valid_states = list(env.MDP[2])

    next_state = env._int2extState([[0,4],[8,4]],[0,0],15)
    env.piecesGridPos = [ cfg['start']  for cfg in env.piecesGridCfg ] # esto deberia estar en env.reset
    env.reset(next_state)
    env.render()
    done = False
    images = []
    i=0
    while done == False:
        action = policy[ valid_states.index(next_state) ]
        #action = policy[next_state]
        print(i, env._ext2intAction(action))
        i+=1
        next_state, reward, done, info = env.step(action)
        env.render()
        time.sleep(0.2)
        #print(env.armsGridPos, env.armsStatus, env.piecesMap)
        #image = np.frombuffer(env.fig.canvas.tostring_rgb(), dtype='uint8')
        #image = image.reshape(env.fig.canvas.get_width_height()[::-1] + (3,))
        #images.append(image)
    
    #imageio.mimsave('YuMi_4pieces_dist55.gif', images, duration=0.2)

