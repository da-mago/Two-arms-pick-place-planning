#include <stdio.h> 
#include <stddef.h> 
#include <stdint.h> 
#include <stdlib.h>
#include "c_bfs.h"

/* How to compile:
 *
 * - Linux   
 *   $ gcc -fPIC -shared -o c_bfs.so c_bfs.c
 *
 * - Windows (x64 Native Tools Command Prompt for VS 2022)
 *   $ cl /LD bfs.c /Fe:bfs.dll 
 */

MYLIB_EXPORT 
void BFS(uint32_t  init_state,
         uint32_t  n_states,
         uint32_t  n_actions,
         uint32_t *states,
         int16_t  *rewards,
         uint32_t *path,
         uint32_t *path_len,
         uint16_t *actions)
{
  uint8_t  *frontier;
  uint8_t  *visited;
  uint32_t *frontier_fifo;
  uint32_t *state_previous;
  uint16_t *action_previous;
  uint32_t tmp = 0;
  uint32_t rp = 0, wp = 0;
  uint32_t next_state;
  uint32_t state;
  uint32_t path_idx = 0;

  frontier = calloc(n_states, 1);
  visited  = calloc(n_states, 1);
  frontier_fifo   = calloc(n_states, 4);
  state_previous  = calloc(n_states, 4);
  action_previous = calloc(n_states, 2);

  if ((frontier == NULL) || (visited == NULL) || (frontier_fifo == NULL) ||
      (state_previous == NULL) || (action_previous == NULL)) 
          return;

  frontier_fifo[wp++] = init_state;

  while (rp != wp)
  {
      state = frontier_fifo[rp];
      frontier[state] = 0;
      visited[state]  = 1;
      if (++rp == n_states) rp = 0;
  
      for (uint32_t action = 0; action < n_actions; action++)
      {
          int16_t r;

          r = rewards[state*n_actions + action];
          if (r < (-1))
          {
              // Discard it (bad action)
              continue;
          }
          else if (r > 50)
          {
              // Done
              next_state = states[state*n_actions + action];
              state_previous[next_state] = state;
              action_previous[next_state] = action;
              // Compute path
              while (1)
              {
                  path[path_idx] = next_state;
                  actions[path_idx] = action_previous[next_state];
                  path_idx++;
                  tmp++;
                  if (state_previous[next_state] == 0)
                  {
                      printf("not found\n");
                      break;
                  }
                  next_state = state_previous[next_state];
    
                  if (next_state == init_state)
                  {
                      // not including initial state
                      *path_len = path_idx;
                      path[path_idx++] = next_state;
                      actions[path_idx++] = action_previous[next_state];
                      break;
                  }
              }
              goto release;
          }
          else
          {
              next_state = states[state*n_actions + action];
              if ((visited[next_state] == 0) && (frontier[next_state] == 0))
              {
                  frontier[next_state] = 1;
                  frontier_fifo[wp] = next_state;
                  state_previous[next_state] = state;
                 action_previous[next_state] = action;
                  if (++wp == n_states) wp = 0;
              }
          }
      }
  }

release:
  free(frontier);
  free(visited);
  free(frontier_fifo);
  free(state_previous);
  free(action_previous);
}
