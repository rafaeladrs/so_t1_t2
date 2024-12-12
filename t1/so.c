// so.c
// sistema operacional
// simulador de computador
// so24b

// INCLUDES {{{1
#include "so.h"
#include "console.h"
#include "dispositivos.h"
#include "err.h"
#include "es.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"
#include "process.h"
#include "cpu.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

// CONSTANTES E TIPOS {{{1
// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas
#define MAX_PROCESSES 4
#define SCHEADULER_QUANTUM 1

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;
  // T1:
  Process* process_table[MAX_PROCESSES];
  int current_process;
  int quantum;
};

Process* process_table_find(so_t* os, int pid);
void process_receive_input(es_t* io, Process* proc);
void process_send_output(es_t* io, Process* proc);

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);

// CRIAÇÃO {{{1

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->quantum = SCHEADULER_QUANTUM;

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // coloca o tratador de interrupção na memória
  // quando a CPU aceita uma interrupção, passa para modo supervisor, 
  //   salva seu estado à partir do endereço 0, e desvia para o endereço
  //   IRQ_END_TRATADOR
  // colocamos no endereço IRQ_END_TRATADOR o programa de tratamento
  //   de interrupção (escrito em asm). esse programa deve conter a 
  //   instrução CHAMAC, que vai chamar so_trata_interrupcao (como
  //   foi definido acima)
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != IRQ_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  free(self);
}


// TRATAMENTO DE INTERRUPÇÃO {{{1

// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
//   interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
//   a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
//   da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupção em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupção
static int so_trata_interrupcao(void *argC, int reg_A)
{
  so_t *self = argC;
  irq_t irq = reg_A;
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupção
  so_trata_irq(self, irq);
  // faz o processamento independente da interrupção
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  so_escalona(self);
  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{ // T1: Salva o estado da CPU no descritor do processo corrente.
  if (!(self->current_process >= 0 && self->current_process < MAX_PROCESSES)) return; 
  if (!self->process_table[self->current_process]) return;

  Process_Context ctx;
  mem_le(self->mem, IRQ_END_PC, &ctx.pc);
  mem_le(self->mem, IRQ_END_A, &ctx.a);
  mem_le(self->mem, IRQ_END_X, &ctx.x);
  mem_le(self->mem, IRQ_END_erro, (int*) &ctx.err);
  self->process_table[self->current_process]->context = ctx;
}

static void so_trata_pendencias(so_t *self)
{ // T1: Trata pendências e contabilidade.
  
  for (int i = 0; i < MAX_PROCESSES; i++) {
    Process* proc = self->process_table[i];
    if (!proc) continue;
    if (proc->state != Process_State_BLOCKING) continue;

    switch (proc->blocking.on) {
      case Process_Blocking_On_INPUT: {
        int state = 0;  
        es_le(self->es, proc->blocking.id, &state);
        if (state) {
          process_receive_input(self->es, proc);
          proc->state = Process_State_READY;
          proc->blocking.on = Process_Blocking_On_NOT_BLOCKING;
        }
      } break;

      case Process_Blocking_On_OUTPUT: {
        int state = 0;  
        es_le(self->es, proc->blocking.id, &state);
        if (state) {
          process_send_output(self->es, proc);
          proc->state = Process_State_READY;
          proc->blocking.on = Process_Blocking_On_NOT_BLOCKING;
        }
      } break;

      case Process_Blocking_On_PROCESS: {
        Process* target = process_table_find(self, proc->blocking.id);
        if (target && target->state == Process_State_TERMINATED) {
          proc->blocking.on = Process_Blocking_On_NOT_BLOCKING;
          proc->state = Process_State_READY;
        }
      } break;

      default: break;
    }
  }

  for (int i = 0; i < MAX_PROCESSES; i++) {
    Process* proc = self->process_table[i];
    if (!proc) continue;

    if (proc->state == Process_State_TERMINATED) {
      console_printf("DESTROYING PROCESS %d\n", proc->pid);
      // process_destroy(proc);
      // self->process_table[i] = NULL;
    }
  }
}

static void so_escalona(so_t *self)
{ // T1: Escolhe o próximo processo a ser executado.
  // Só escalona se o quantum do processo atual terminou.
  console_printf("Escalonando...");
  
  int next_process = (self->current_process + 1) % MAX_PROCESSES;
  bool found = false;
  for (int i = 0; i < MAX_PROCESSES; i++, next_process = (next_process + 1) % MAX_PROCESSES) {
    if (!self->process_table[next_process]) continue;
    if (self->process_table[next_process]->state & (Process_State_READY | Process_State_RUNNING)) {
      found = true;
      break;
    };
  }

  if (!found) {
    console_printf("NENHUM PROCESSO DISPONÍVEL!");
  }

  // Se o processo anterior estava em 'Running', vai para 'Ready'.
  Process* previous = self->process_table[self->current_process];
  if (previous && previous->state == Process_State_RUNNING) {
    previous->state = Process_State_READY;
  }

  // Processo escalonado entra em 'Running'.
  if (found) self->process_table[next_process]->state = Process_State_RUNNING;

  if (found) self->current_process = next_process;
  else self->current_process = -1;
  self->quantum = SCHEADULER_QUANTUM;
}

static int so_despacha(so_t *self)
{ // T1: Salva o contexto do novo processo.
  for (int i = 0; i < MAX_PROCESSES; i++) {
    console_printf("[%d, %d]", (self->process_table[i]) ? self->process_table[i]->pid : -1,
                   (self->process_table[i]) ? self->process_table[i]->state : -1);
  }

  if (self->erro_interno) {
    console_printf("ERRO INTERNO!");
    return self->erro_interno;
  }

  if (self->current_process >= 0) {
    Process_Context ctx;
    ctx = self->process_table[self->current_process]->context;
    mem_escreve(self->mem, IRQ_END_PC, ctx.pc);
    mem_escreve(self->mem, IRQ_END_A, ctx.a);
    mem_escreve(self->mem, IRQ_END_X, ctx.x);
  } 

  return 0;
}

// TRATAMENTO DE UMA IRQ {{{1

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_irq_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupção que está acontecendo, e atende de acordo
  switch (irq) {
    case IRQ_RESET:
      so_trata_irq_reset(self);
      break;
    case IRQ_SISTEMA:
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      so_trata_irq_relogio(self);
      break;
    default:
      so_trata_irq_desconhecida(self, irq);
  }
}

// Interrupção gerada uma única vez, quando a CPU inicializa.
static void so_trata_irq_reset(so_t *self)
{ // T1: Inicializa a tabela de processos com processo para 'init'.
  memset(self->process_table, 0, sizeof(self->process_table));

  // Carrega o programa 'init' na memória.
  int ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  Process* ps = process_create(ender, D_TERM_A_TECLADO, D_TERM_A_TELA);
  self->current_process = 0;
  self->process_table[self->current_process] = ps;

  ps->context.pc = ender;
  ps->state = Process_State_READY;
}

static void so_trata_irq_err_cpu(so_t *self)
{ // T1: Obtém código do erro do descritor e mata o processo corrente.
  if (self->current_process >= 0 && self->current_process < MAX_PROCESSES) {
    Process* ps = self->process_table[self->current_process];
    err_t err = self->process_table[self->current_process]->context.err;
    console_printf("SO: Erro na CPU: %s", err_nome(err));
    ps->state = Process_State_TERMINATED;
  }

  self->erro_interno = true;
}

static void so_trata_irq_relogio(so_t *self)
{
  // rearma o interruptor do relógio e reinicializa o timer para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }

  // T1: Decrementa o quantum do processo corrente.
  self->quantum--;
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}

// CHAMADAS DE SISTEMA {{{1

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  int id_chamada = self->process_table[self->current_process]->context.a;
  console_printf("SO: chamada de sistema %d", id_chamada);
  switch (id_chamada) {
    case SO_LE:
      console_printf("Syscall LE");
      so_chamada_le(self);
      break;
    case SO_ESCR:
      console_printf("Syscall ESCR");
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      console_printf("Syscall CRIA_PROC");
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      console_printf("Syscall MATA_PROC");
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      console_printf("Syscall ESPERA_PROC");
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);

      // T1: Mata o processo.
      self->process_table[self->current_process]->state = Process_State_TERMINATED;
      
      self->erro_interno = true;
  }
}

void process_receive_input(es_t* io, Process* proc) {
  int data;
  if (es_le(io, proc->in, &data) == ERR_OK) {
    proc->context.a = data;
  } else {
    proc->state = Process_State_TERMINATED;
  }
}

static void so_chamada_le(so_t *self)
{ // T1: Realiza a leitura se o dispositivo estiver disponível, senão bloqueia o processo.
  Process* proc = self->process_table[self->current_process];

  int state;
  if (es_le(self->es, proc->in + 1, &state) != ERR_OK) {
    proc->state = Process_State_TERMINATED;
  } else if (!state) {
    // Dispositivo indisponível, bloqueia o processo.
    proc->state = Process_State_BLOCKING;
    proc->blocking = (Process_Blocking) {
      .on = Process_Blocking_On_INPUT,
      .id = proc->in + 1,
    };
  } else {
    // Dispositivo disponível
    process_receive_input(self->es, proc);
  }
}

void process_send_output(es_t* io, Process* proc) {
  if (es_escreve(io, proc->out, proc->context.x) == ERR_OK) {
    proc->context.a = 0;
  } else {
    proc->state = Process_State_TERMINATED;
  }
}

static void so_chamada_escr(so_t *self)
{ // T1: Realiza a escrita se o dispositivo estiver disponível, senão bloqueia o processo.
  Process* proc = self->process_table[self->current_process];
  int state;

  if (es_le(self->es, proc->out + 1, &state) != ERR_OK) {
    proc->state = Process_State_TERMINATED;
  } else if (state == 0) {
    // Dispositivo indisponível, bloqueia o processo.
    proc->state = Process_State_BLOCKING;
    proc->blocking = (Process_Blocking) {
      .on = Process_Blocking_On_OUTPUT,
      .id = proc->out + 1,
    };
  } else {
    // Dispositivo disponível
    process_send_output(self->es, proc);
  }
}

static void so_chamada_cria_proc(so_t *self)
{ // T1: Cria novo processo.
  Process* proc = self->process_table[self->current_process];
  int filename_address = proc->context.x;

  char filename[256];
  if (!copia_str_da_mem(256, filename, self->mem, filename_address)) goto fail;

  int table_entry = -1;
  for (int i = 0; i < MAX_PROCESSES; i++) {
    if (self->process_table[i] == NULL) {
      table_entry = i;
    }
  }
  if (table_entry == -1) goto fail;

  int program_address = so_carrega_programa(self, filename);    
  if (program_address < 0) goto fail;

  // Seleciona teclado e tela de A até D dependendo da entrada na tabela de processos.
  dispositivo_id_t in = (table_entry % 4) * 4;
  dispositivo_id_t out = in + 2;

  Process* new_proc = process_create(program_address, in, out);
  new_proc->context.pc = program_address;
  new_proc->state = Process_State_READY;
  self->process_table[table_entry] = new_proc;

  proc->context.a = new_proc->pid;

  return;
  fail:
  console_printf("Criação de processo falhou!");
  proc->context.a = -1;
}

static void so_chamada_mata_proc(so_t *self)
{ // T1: Mata um processo.
  Process* proc = self->process_table[self->current_process];
  int pid = proc->context.x;

  // Mata a si mesmo.
  if (pid == 0) {
    proc->state = Process_State_TERMINATED;
    return;
  }

  Process* target = process_table_find(self, pid);
  if (target) {
    target->state = Process_State_TERMINATED;
    proc->context.a = 0;
  } else {
    // Não encontrado na tabela.
    proc->state = Process_State_TERMINATED;
  }
}

static void so_chamada_espera_proc(so_t *self)
{ // T1: Bloqueia o processo corrente até a morte do outro.
  Process* proc = self->process_table[self->current_process];
  Process* target = process_table_find(self, proc->context.x);

  if (target && target->pid != proc->pid) {
    proc->state = Process_State_BLOCKING;
    proc->blocking = (Process_Blocking) {
      .on = Process_Blocking_On_PROCESS,
      .id = target->pid,
    };
    proc->context.a = 0;
  }  else {
    proc->state = Process_State_TERMINATED;
  }
}

// CARGA DE PROGRAMA {{{1

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}

// ACESSO À MEMÓRIA DOS PROCESSOS {{{1

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// T1: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

Process* process_table_find(so_t* os, int pid) {
  for (int i = 0; i < MAX_PROCESSES; i++) {
    Process* proc = os->process_table[i];
    if (proc && proc->pid == pid) return proc;
  }
  return NULL;
}

// vim: foldmethod=marker
