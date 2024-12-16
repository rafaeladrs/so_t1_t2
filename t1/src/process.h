#ifndef PROCESS_H
#define PROCESS_H

#include "err.h"
#include "dispositivos.h"

typedef enum {
    Process_State_NEW = 0,
    Process_State_READY = 1 << 0,
    Process_State_RUNNING = 1 << 1,
    Process_State_BLOCKING = 1 << 2,
    Process_State_TERMINATED = 1 << 3,
} Process_State;

typedef enum {
    Process_Blocking_On_NOT_BLOCKING = 0,
    Process_Blocking_On_INPUT = 1 << 0,
    Process_Blocking_On_OUTPUT = 1 << 1,
    Process_Blocking_On_PROCESS = 1 << 2,
} Process_Blocking_On;

typedef struct {
    Process_Blocking_On on;
    int id; // IO device or external process identififer.
} Process_Blocking;

typedef struct {
    int pc;
    int a;
    int x;
    err_t err;
} Process_Context;

typedef struct {
    int pid;
    float priority;
    Process_State state;
    Process_Blocking blocking;
    Process_Context context;
    dispositivo_id_t in;
    dispositivo_id_t out;
} Process;

Process* process_create(int pc, dispositivo_id_t in, dispositivo_id_t out);
void process_destroy(Process* ps);

#endif // PROCESS_H
