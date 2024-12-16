#include "process.h"

#include <stdlib.h>

int process_counter = 0;

Process* process_create(int pc, dispositivo_id_t in, dispositivo_id_t out) {
    Process* ps = malloc(sizeof(Process));

    *ps = (Process) {
        .pid = process_counter++,
        .priority = 0.5,
        .context.pc = pc,
        .in = in,
        .out = out,
    };

    return ps;
}

void process_destroy(Process* ps) {
    free(ps);
}
