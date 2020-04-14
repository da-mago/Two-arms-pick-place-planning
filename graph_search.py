import numpy as np
import envYumi
import time
import sys

found = False
goal_path = []

def graph_search(init_state, algorithm=0):
    ''' Apply this search algorithm:
        - bfs (0)
        - dfs (1)
    '''
    goal_state = 0
    backTrace = {states_idx[init_state]: None}
    frontier  = np.zeros((states_len), dtype=np.int32)
    visited   = np.zeros((states_len), dtype=np.int8)
    frontier[0] = states_idx[init_state]
    frontier_head = 0
    frontier_tail = frontier_head + 1
    visited_cnt  = 0
    done = False
    cnt = 0
    while not done:
        state_idx = frontier[frontier_head]
        if algorithm == 0: frontier_head += 1
        else:              frontier_head -= 1
        for next_state, reward in zip(next_states[state_idx], rewards[state_idx]):
            idx = states_idx[next_state]
            #idx = next_state
            if reward > 10:
                goal_state = idx
                backTrace[idx] = state_idx
                print('Goal reached!')
                done = True
                break
            elif reward > -5:
                #if idx not in visited:
                if visited[idx] == 0:
                    if algorithm == 0: # BFS
                        if idx not in frontier[frontier_head:frontier_tail]:
                            frontier[frontier_tail] = idx
                            frontier_tail += 1
                            backTrace[idx] = state_idx
                    else:              # DFS
                        if algorithm == 1 and idx not in frontier[:frontier_head+1]:
                            frontier_head += 1
                            frontier[frontier_head] = idx
                            backTrace[idx] = state_idx
        visited[state_idx] = 1

        #visited_cnt += 1
        ##if len(visited) % 1300 == 0:
        #if visited_cnt % 1300 == 0:
        #    print(cnt)
        #    cnt += 1

        if frontier_head == frontier_tail:
            print('Goal NOT found!')
            done = True

    # Build the path
    path = []
    prior_state = goal_state
    while prior_state != None:
        path.insert(0, prior_state)
        prior_state = backTrace[prior_state]

    return path

if __name__ == "__main__":

    env = envYumi.EnvYuMi()
    env.reset()

    next_states = env.MDP[0]
    rewards     = env.MDP[1]
    states_val  = env.MDP[2]
    states_len  = len(env.MDP[2])

    states_idx = {}
    for i, item in enumerate(states_val):
        states_idx[item] = i

    init_state = env._int2extState([[0,0],[6,2]], [0,0], 15)

    # Find the optimal solution
    #
    path = graph_search(init_state, 0) # 0: BFS, 1: DFS

    #sys.exit()

    # Show the solution
    for state_idx in path:
        state = states_val[state_idx]
        env.reset(state)
        armsGridPos, armsStatus, piecesMap = env._ext2intState(state)
        for status, pos in zip(armsStatus, armsGridPos):
            if status != 0:
                env.piecesGridPos[status-1] = pos
        env.render()
        time.sleep(0.1)

