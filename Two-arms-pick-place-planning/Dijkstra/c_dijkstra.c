#include <stdio.h> 
#include <stddef.h> 
#include <stdint.h> 
#include <stdlib.h>
#include "c_dijkstra.h"

/* How to compile:
 *
 * - Linux   
 *   $ gcc -fPIC -shared -o c_dijkstra.so c_dijkstra.c
 *
 * - Windows (x64 Native Tools Command Prompt for VS 2022)
 *   $ cl /LD c_dijkstra.c /Fe:c_dijkstra.dll 
 */

#define MAX_COST 0x7FFF

typedef struct frontier {
    uint32_t next;
    int16_t  g;
} frontierQ;

int frontierInsert(uint32_t state, int16_t g, uint32_t *frontier_head, frontierQ *frontier_q) {

    uint32_t idx = *frontier_head;
    uint32_t prev_idx;

    /* Keep states ordered by cost (g).
     * For states with the same g, keep the insertion order, so insert in the end
     */

    if (idx == -1) {
        // First and unique state in the ueue
        frontier_q[state].next = -1;
        frontier_q[state].g = g;
        *frontier_head = state;

    } else {
        while (idx != -1) {
            if (g < frontier_q[idx].g) {
                if (frontier_q[idx].g == MAX_COST) printf("PERO PERO\n");
                frontier_q[state].next = idx;
                frontier_q[state].g = g;
                if (idx == *frontier_head) *frontier_head = state;            // First state in the queue
                else                       frontier_q[prev_idx].next = state; //somewhere in the middle of the queue

                return 0;
            }
            prev_idx = idx;
            idx = frontier_q[idx].next;
        }

        // Last in the queue
        frontier_q[state].next = -1;
        frontier_q[state].g = g;
        frontier_q[prev_idx].next = state;
    }

    return 0;
}

////int getStateFromFrontier(uint32_t  n_states, uint8_t  *frontier, int32_t  *cost_g, uint32_t *state)
////{
////    int32_t min_cost = MAX_COST;
////    int ret = -1;
////
////    // Get the state in the frontier with the lowest cost
////    uint32_t cnt = 0;
////    for (uint32_t i = 0; i < n_states; i++)
////	if (frontier[i]) {
////	   ret = 0;
////           if (cost_g[i] < min_cost) {
////               min_cost = cost_g[i];
////               *state = i; 
////           }
////	   cnt++;
////	}
////
////    //printf("cnt %d %d ", cnt, *state);
//////fflush(stdout); 
////    return ret;
////}

MYLIB_EXPORT 
int Dijkstra(uint32_t  init_state,
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
//  uint32_t *frontier_fifo;
  int32_t  *cost_g;
  uint32_t *state_previous;
  uint16_t *action_previous;
  uint32_t tmp = 0;
  uint32_t rp = 0, wp = 0;
  uint32_t next_state;
  uint32_t state;
  uint32_t path_idx = 0;
  frontierQ *frontier_q;
  uint32_t frontier_head = -1;

  frontier = calloc(n_states, 1);
  visited  = calloc(n_states, 1);
//  frontier_fifo   = calloc(n_states, 4);
  cost_g   = calloc(n_states, 4);
  frontier_q      = calloc(n_states, sizeof(frontierQ)); // queue next state and cost (g)
  state_previous  = calloc(n_states, 4);
  action_previous = calloc(n_states, 2);

  if ((frontier == NULL) || (visited == NULL) || (cost_g == NULL) ||  /*(frontier_fifo == NULL) ||*/
      (state_previous == NULL) || (action_previous == NULL) || (frontier_q == NULL))
          return -1;

  //printf("Dijkstra %d\n", n_states);
//  frontier_fifo[wp++] = init_state;
////  for (uint32_t i = 0; i < n_states; i++)
////    cost_g[i] = MAX_COST;
////  cost_g[init_state] = 0;
////  frontier[init_state] = 1;

  for (uint32_t i = 0; i < n_states; i++) {
      frontier_q[i].g = MAX_COST;
      frontier_q[i].next = -1;
  }
  frontier_q[init_state].g = 0;
  frontier_head = init_state;
  //printf("Dijkstra %d %d\n", n_states, init_state);

  for (uint32_t i = 0; i < n_states; i++)
  {
      int err;
      int16_t g;
//      state = frontier_fifo[rp];
////      err = getStateFromFrontier(n_states, frontier, cost_g, &state);
////      if (err == -1)
////      {
////	      printf("TREE done, and path not found\n");
////	      return -1;
////      }
//      if ((i % (13025)) == 0) { printf("T:%d %d", i, state); fflush(stdout); }
      state = frontier_head;
      //printf("STATE %d\n", state);
      if (state == -1)
      {
	      printf("TREE done, and path not found\n");
	      return -1;
      }
      g = frontier_q[state].g;
      frontier_head = frontier_q[state].next;
      frontier_q[state].g = MAX_COST;
      frontier_q[state].next = -1;
      //printf("FHEAD %d\n", frontier_head);

////      frontier[state] = 0;
      visited[state]  = 1;
//      if (++rp == n_states) rp = 0;
  
      for (uint32_t action = 0; action < n_actions; action++)
      {
          int16_t r;

          r = rewards[state*n_actions + action];
//	  printf("R %d", r);
          if (r < (-10))
          {
              // Discard it (bad action)
              continue;
          }
          else if (r > 50)
          {
		//printf("LOOPED %d\n",i);
              // Done (algorithm customized for the case where all costs are the same)
              next_state = states[state*n_actions + action];
              state_previous[next_state] = state;
              action_previous[next_state] = action;
              // Compute path
              while (1)
              {
		      //printf("%d ", next_state);
                  path[path_idx] = next_state;
                  actions[path_idx] = action_previous[next_state];
                  path_idx++;
                  tmp++;
                  if (state_previous[next_state] == 0)
                  {
                      printf("not found %d\n", next_state);
//                      break;
		      return -1;
                  }
                  next_state = state_previous[next_state];

                  if (next_state == init_state)
                  {
			printf("Done!\n");
                      // not including initial state
                      *path_len = path_idx;
                      path[path_idx++] = next_state;
                      actions[path_idx++] = action_previous[next_state];
              	      goto release;
                  }
              }
          }
          else
          {
              int32_t cost;
              int ret;

              next_state = states[state*n_actions + action];
              if (visited[next_state] == 0)
              { 
////		  if ((cost_g[next_state] != MAX_COST) && (frontier[next_state] == 0)) {printf("WTF!!!!\n"); return -1;}
////                  cost = cost_g[state] + (-r);
                  cost = g + (-r);
////                  if (cost < cost_g[next_state]) {
                 if (cost < frontier_q[next_state].g) {

////                      frontier[next_state] = 1;
                      ret = frontierInsert(next_state, cost, &frontier_head, frontier_q);
                      if (ret == -1) {printf("ERROR INSERTING\n"); return -1;}
                     // printf("F %d ", frontier_head);
	             // printf("C %d %d %d %d ", next_state, cost, g, frontier_q[next_state].g);
//                     uint32_t idx = frontier_head;
//                     while (idx != -1) {
//                         printf("%d %d ", idx, frontier_q[idx].g);
//                         idx = frontier_q[idx].next;
//                     } 
//                     printf("\n");
//                  frontier_fifo[wp] = next_state;
                      state_previous[next_state] = state;
                      action_previous[next_state] = action;
//                  if (++wp == n_states) wp = 0;
  		  }
              }
          }
      }
  }

release:
  free(frontier);
  free(visited);
//  free(frontier_fifo);
  free(cost_g);
  free(state_previous);
  free(action_previous);
  //printf("BYE\n");

  return 0;
}
