#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
    #define MYLIB_EXPORT __declspec(dllexport) // Windows DLL
#else
    #define MYLIB_EXPORT                       // Linux so
#endif

MYLIB_EXPORT int Dijkstra(uint32_t init_state, uint32_t n_states, uint32_t n_actions, uint32_t* states, int16_t* rewards, uint32_t* path, uint32_t* path_len, uint16_t* actions);

#ifdef __cplusplus
}
#endif

#endif

