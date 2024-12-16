#include "process.h"

#include <stdlib.h>
#include <assert.h>

int process_counter = 0;

Process* process_create(dispositivo_id_t in, dispositivo_id_t out) {
    Process* ps = malloc(sizeof(Process));
    assert(ps);

    *ps = (Process) {
        .pid = process_counter++,
        .priority = 0.5,
        .context.pc = 0,
        .in = in,
        .out = out,
        .page_table = tabpag_cria(),
    };

    return ps;
}

void process_destroy(Process* proc) {
    tabpag_destroi(proc->page_table);
    free(proc);
}
