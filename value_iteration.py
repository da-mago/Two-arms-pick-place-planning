import numpy as np
import sys
import pickle
import envYumi
import time    # sleep
import imageio # mimsave

def value_iteration(env, V, theta=0.0001, discount_factor=1.0):
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
    

    state_idx = {}
    for i, item in enumerate(env.MDP[2]):
        state_idx[item] = i

    ## Get MDP (or compute MDP the first time)
    #import gc
    #import cPickle
    #gc.disable()
    #try:
    #    mdp = cPickle.load( open( "mdp.bin", "rb" ))
    #except:
    #    mdp = env.get_MDP()
    #    cPickle.dump(mdp, open( "mdp.bin", "wb" ), protocol=-1  )
    #gc.enable()
    #print('MDP')

    #cnt = 0
    #states_per_level = env.get_states_per_level()
    #print(states_per_level)
    #for level in states_per_level:
    #    #break
    #    for state_base in level:

    #        while True:
    #            # Stopping condition
    #            delta = 0
    #            V_cp = np.copy(V)

    #            # Update each state...
    #            for s in range(state_base, state_base+625):
    #                if env.is_state_invalid(s):
    #                    continue

    #                #print(env._state_decode(state_base))
    #                # Do a one-step lookahead to find the best action
    #                VA = one_step_lookahead(s, V_cp)
    #                best_action_value = np.max(VA)
    #                # Calculate delta across all states seen so far
    #                delta = max(delta, np.abs(best_action_value - V_cp[s]))
    #                # Update the value function. Ref: Sutton book eq. 4.10. 
    #                V[s] = best_action_value        
    #                #if V[s] != V_cp[s] and s == 774:
    #                #    print(s, V_cp[s], V[s])
    #            # Check if we can stop 
    #            if delta < theta:
    #                #ppython fastrint(cnt)
    #                break
    #            else:
    #                #print(cnt, delta)
    #                cnt += 1
    #    #break

    done = False
    cnt = 0
    while not done:
        # Stopping condition
        updated = 0
        delta = 0
        done = True
        V_cp = np.copy(V)

        # For every valid state
        for i in range(states_len):
            # Update the value function
            #              reward                            next_state
            next_states_idx = np.array([state_idx[next_state] for next_state in env.MDP[0][i]], dtype=np.int32)
            V[i] = np.max( env.MDP[1][i] + discount_factor*V[next_states_idx] ) # one-step lookahead

            # Calculate delta across all states seen so far
            curr_delta = np.abs(V[i] - V_cp[i])
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
        print(cnt, updated, delta)
        
    print('Done')
    
    # Compute optimal policy (based on optimal value function)
    #                    <---------------  one-step lookahed ---------------------->
    #policy = np.array( [ np.argmax(env.MDP[1][s] + discount_factor*V[env.MDP[0][s]]) for s in range(env.nS) ], dtype=np.int8)
    policy = np.zeros(states_len, dtype=np.int8)
    for i in range(states_len):
        next_states_idx = np.array([state_idx[next_state] for next_state in env.MDP[0][i]], dtype=np.int32)
        policy[i] = np.argmax(env.MDP[1][i] + discount_factor*V[next_states_idx])
    
    return policy, V, list(env.MDP[2])

if __name__ == "__main__":

    #pieces_cfg = [{'start' : [-0.5, -0.5, 0],  # Piece 1
    #               'end'   : [ 0.5, -1.5, 0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.3, -0.8, 0],  # Piece 2
    #               'end'   : [ 1,   -1,   0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.3, -1,   0],  # Piece 3
    #               'end'   : [ 0.3, -1,   0],
    #               'nArms' : 1
    #              },
    #              {'start' : [-0.6, -1.3, 0],  # Piece 4
    #               'end'   : [ 0.6, -0.8, 0],
    #               'nArms' : 1
    #              }]
    #              #},
    #              #{'start' : [-0.7, -0.7, 0],  # Piece 5
    #              # 'end'   : [ 1.2, -0.4, 0],
    #              # 'nArms' : 1
    #              #},
    #              #{'start' : [-0.8, -1,   0],  # Piece 6
    #              # 'end'   : [ 0.5, -1.2,   0],
    #              # 'nArms' : 1
    #              #},
    #              #{'start' : [-0.9, -0.5, 0],  # Piece 7
    #              # 'end'   : [ 1.2, -0.8, 0],
    #              # 'nArms' : 1
    #              #}]
    #env = envYumi.EnvYuMi(pieces_cfg)
    env = envYumi.EnvYuMi()
    env.reset()

    states_len = len(env.MDP[2])
    V = np.zeros(states_len, dtype=np.float)
    print(states_len)

    # Train
    if True:
        V = pickle.load( open( "value.bin", "rb" ) )

        policy, v, valid_states = value_iteration(env, V, discount_factor=0.95)

        #pickle.dump( v,      open( "value.bin", "wb" ) )
        #pickle.dump( policy, open( "policy.bin", "wb" ) )
    else:
        policy = pickle.load( open( "policy.bin", "rb" ) )

    valid_states = list(env.MDP[2])

    next_state = env._int2extState([[0,4],[8,4]],[0,0],15)
    env.piecesGridPos = [ cfg['start']  for cfg in env.piecesGridCfg ] # esto deberia estar en env.reset
    env.reset(next_state)
    env.render()
    done = False
    images = []
    while done == False:
        action = policy[ valid_states.index(next_state) ]
        #action = policy[next_state]
        print(env._ext2intAction(action))
        next_state, reward, done, info = env.step(action)
        env.render()
        time.sleep(0.2)
        #print(env.armsGridPos, env.armsStatus, env.piecesMap)
        #image = np.frombuffer(env.fig.canvas.tostring_rgb(), dtype='uint8')
        #image = image.reshape(env.fig.canvas.get_width_height()[::-1] + (3,))
        #images.append(image)
    
    #imageio.mimsave('YuMi_4pieces_dist55.gif', images, duration=0.2)

