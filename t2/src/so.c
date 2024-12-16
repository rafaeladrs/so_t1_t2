// so.c
// sistema operacional
// simulador de computador
// so24b

// INCLUDES {{{1
#include "so.h"
#include "cpu.h"
#include "dispositivos.h"
#include "irq.h"
#include "memoria.h"
#include "mmu.h"
#include "programa.h"
#include "tabpag.h"
#include "process.h"

#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>

// CONSTANTES E TIPOS {{{1
// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas
#define MAX_PROCESSES 4
#define SCHEADULER_QUANTUM 2 // Em interrupções do clock.
#define NO_PROCESS_RUNNING -1

// Não tem processos nem memória virtual, mas é preciso usar a paginação,
//   pelo menos para implementar relocação, já que os programas estão sendo
//   todos montados para serem executados no endereço 0 e o endereço 0
//   físico é usado pelo hardware nas interrupções.
// Os programas estão sendo carregados no início de um quadro, e usam quantos
//   quadros forem necessárias. Para isso a variável quadro_livre contém
//   o número do primeiro quadro da memória principal que ainda não foi usado.
//   Na carga do processo, a tabela de páginas (deveria ter uma por processo,
//   mas não tem processo) é alterada para que o endereço virtual 0 resulte
//   no quadro onde o programa foi carregado. Com isso, o programa carregado
//   é acessível, mas o acesso ao anterior é perdido.

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  mem_t* mem_secundaria;
  mmu_t *mmu;
  es_t *es;
  console_t *console;
  bool erro_interno;
  // t1: tabela de processos, processo corrente, pendências, etc
  Process* process_table[MAX_PROCESSES];
  int current_process;
  int quantum;
  // primeiro quadro da memória que está livre (quadros anteriores estão ocupados)
  // t2: com memória virtual, o controle de memória livre e ocupada é mais
  // completo que isso
  int quadro_livre;
  // uma tabela de páginas para poder usar a MMU
  // t2: com processos, não tem esta tabela global, tem que ter uma para
  //     cada processo
  // tabpag_t *tabpag_global;
};

Process* process_table_find(so_t* os, int pid);
void process_receive_input(es_t* io, Process* proc);
void process_send_output(es_t* io, Process* proc);

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// no t2, foi adicionado o 'processo' aos argumentos dessas funções
// carrega o programa na memória virtual de um processo; retorna end. inicial
static int so_carrega_programa(so_t *self, Process* processo,
                               char *nome_do_executavel);
// copia para str da memória do processo, até copiar um 0 (retorna true) ou tam bytes
static bool so_copia_str_do_processo(so_t *self, int tam, char str[tam],
                                     int end_virt, Process* processo);

// CRIAÇÃO {{{1


so_t *so_cria(cpu_t *cpu, mem_t *mem, mem_t* mem_secundaria, mmu_t *mmu,
              es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  assert(self != NULL);

  self->cpu = cpu;
  self->mem = mem;
  self->mem_secundaria = mem_secundaria;
  self->mmu = mmu;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->current_process = NO_PROCESS_RUNNING;
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
  int ender = so_carrega_programa(self, NULL, "trata_int.maq");
  if (ender != IRQ_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  // inicializa a tabela de páginas global, e entrega ela para a MMU
  // t2: com processos, essa tabela não existiria, teria uma por processo, que
  //     deve ser colocada na MMU quando o processo é despachado para execução
  // self->tabpag_global = tabpag_cria();
  // mmu_define_tabpag(self->mmu, self->tabpag_global);
  // define o primeiro quadro livre de memória como o seguinte àquele que
  //   contém o endereço 99 (as 100 primeiras posições de memória (pelo menos)
  //   não vão ser usadas por programas de usuário)
  // t2: o controle de memória livre deve ser mais aprimorado que isso
  self->quadro_livre = 99 / TAM_PAGINA + 1;
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

  // T1: Se nenhum processo foi escalonado, parte pra espera ativa.
  while (!self->erro_interno && self->current_process == NO_PROCESS_RUNNING) {
    console_tictac(self->console);
    so_trata_pendencias(self);
    so_escalona(self);
  }

  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{ // T1: Salva o estado da CPU no descritor do processo corrente.
  if (self->current_process == NO_PROCESS_RUNNING || !self->process_table[self->current_process]) return;

  Process_Context ctx;
  mmu_le(self->mmu, IRQ_END_PC, &ctx.pc, supervisor);
  mmu_le(self->mmu, IRQ_END_A, &ctx.a, supervisor);
  mmu_le(self->mmu, IRQ_END_X, &ctx.x, supervisor);
  mmu_le(self->mmu, IRQ_END_erro, (int*) &ctx.err, supervisor);
  self->process_table[self->current_process]->context = ctx;
}

static void so_trata_pendencias(so_t *self)
{ // T1: Trata pendências e contabilidade.

  for (int i = 0; i < MAX_PROCESSES; i++) {
    Process* proc = self->process_table[i];
    if (!proc || proc->state != Process_State_BLOCKING) continue;

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
      console_printf("SO: Destruindo processo %d\n.", proc->pid);
      process_destroy(proc);
      self->process_table[i] = NULL;
    }
  }
}

static void so_escalona(so_t *self)
{ // T1: Escolhe o próximo processo a ser executado.
  // Só escalona se o quantum do processo atual terminou ou não existe processo executando.
  if (self->current_process != NO_PROCESS_RUNNING
  && self->process_table[self->current_process]
  && self->process_table[self->current_process]->state == Process_State_RUNNING
  && self->quantum > 0) return;

  int next_process = (self->current_process + 1) % MAX_PROCESSES;
  bool found = false;
  bool remaining = false;
  for (int i = 0; i < MAX_PROCESSES; i++, next_process = (next_process + 1) % MAX_PROCESSES) {
    Process* proc = self->process_table[next_process];
    if (proc) {
      remaining = true;
      if (proc->state & (Process_State_READY | Process_State_RUNNING)) {
        found = true;
        break;
      }
    }
  }

  if (!remaining) {
    console_printf("SO: Não existem mais processos!");
    // self->erro_interno = true;
  }

  if (found) {
    // Se o processo anterior estava em 'Running', vai para 'Ready'.
    if (self->current_process != NO_PROCESS_RUNNING
    && self->process_table[self->current_process]) {
      Process* previous = self->process_table[self->current_process];
      if (previous->state == Process_State_RUNNING) previous->state = Process_State_READY;
    }

    // Atualiza processo em execução.
    self->current_process = next_process;
    self->process_table[self->current_process]->state = Process_State_RUNNING;
  }
  else {
    self->current_process = NO_PROCESS_RUNNING;
  }

  self->quantum = SCHEADULER_QUANTUM;

  for (int i = 0; i < MAX_PROCESSES; i++)
    console_printf("[%d, %d]", (self->process_table[i]) ? self->process_table[i]->pid : -1,
                   (self->process_table[i]) ? self->process_table[i]->state : -1);
  console_printf("\n");
}

static int so_despacha(so_t *self)
{ // T1: Salva o contexto do novo processo.
  if (self->erro_interno) {
    console_printf("SO: Erro interno!");
    return self->erro_interno;
  }

  Process* proc = self->process_table[self->current_process];
  Process_Context ctx = proc->context;
  mmu_escreve(self->mmu, IRQ_END_PC, ctx.pc, supervisor);
  mmu_escreve(self->mmu, IRQ_END_A, ctx.a, supervisor);
  mmu_escreve(self->mmu, IRQ_END_X, ctx.x, supervisor);
  // T2:
  mmu_define_tabpag(self->mmu, proc->page_table);

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

  Process* proc = process_create(D_TERM_A_TECLADO, D_TERM_A_TELA);
  self->current_process = 0;
  self->process_table[self->current_process] = proc;

  // Carrega o programa 'init' na memória.
  int ender = so_carrega_programa(self, proc, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  // TODO: Algo com proc->page_table?

  // Vai de 'New' para 'Ready'.
  proc->state = Process_State_READY;
}

static void so_trata_irq_err_cpu(so_t *self)
{ // T1: Obtém código do erro do descritor e mata o processo corrente.
  if (self->current_process != NO_PROCESS_RUNNING) {
    Process* ps = self->process_table[self->current_process];
    err_t err = self->process_table[self->current_process]->context.err;
    console_printf("SO: Erro na CPU: %s", err_nome(err));
    ps->state = Process_State_TERMINATED;
  }

  self->erro_interno = true;
}

// interrupção gerada quando o timer expira
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
      so_chamada_le(self);
      break;
    case SO_ESCR:
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);

      // T1: Mata o processo.
      self->process_table[self->current_process]->state = Process_State_TERMINATED;

      self->erro_interno = true;
  }
}

void process_receive_input(es_t* io, Process* proc)
{ // Realiza a leitura, assume dispositivo disponível.
  int data;
  if (es_le(io, proc->in, &data) == ERR_OK) {
    proc->context.a = data;
  } else {
    proc->state = Process_State_TERMINATED;
  }
}

void process_send_output(es_t* io, Process* proc)
{ // Realiza a leitura, assume dispositivo disponível.
  if (es_escreve(io, proc->out, proc->context.x) == ERR_OK) {
    proc->context.a = 0;
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

static void so_chamada_escr(so_t *self)
{ // T1: Realiza a escrita se o dispositivo estiver disponível, senão bloqueia o processo.
  Process* proc = self->process_table[self->current_process];

  int state;
  if (es_le(self->es, proc->out + 1, &state) != ERR_OK) {
    proc->state = Process_State_TERMINATED;
  } else if (!state) {
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
  if (!so_copia_str_do_processo(self, 256, filename, filename_address, proc)) goto fail;

  int table_entry = -1;
  for (int i = 0; i < MAX_PROCESSES; i++) {
    if (self->process_table[i] == NULL) {
      table_entry = i;
    }
  }
  if (table_entry == -1) goto fail;

  // Seleciona teclado e tela de A até D dependendo da entrada na tabela de processos.
  dispositivo_id_t in = (table_entry % 4) * 4;
  dispositivo_id_t out = in + 2;
  Process* new_proc = process_create(in, out);

  int program_address = so_carrega_programa(self, new_proc, filename);
  if (program_address < 0) goto fail;

  new_proc->context.pc = program_address;
  new_proc->state = Process_State_READY;
  self->process_table[table_entry] = new_proc;

  proc->context.a = new_proc->pid;

  return;
  fail:
  console_printf("SO: Criação de processo falhou!");
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

// funções auxiliares
static int so_carrega_programa_na_memoria_fisica(so_t *self, programa_t *programa);
static int so_carrega_programa_na_memoria_virtual(so_t *self, programa_t *programa, Process* processo);

// carrega o programa na memória de um processo ou na memória física se NENHUM_PROCESSO
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, Process* processo,
                               char *nome_do_executavel)
{
  console_printf("SO: carga de '%s'", nome_do_executavel);

  programa_t *programa = prog_cria(nome_do_executavel);
  if (programa == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_carga;
  if (!processo) {
    end_carga = so_carrega_programa_na_memoria_fisica(self, programa);
  } else {
    end_carga = so_carrega_programa_na_memoria_virtual(self, programa, processo);
  }

  prog_destroi(programa);
  return end_carga;
}

static int so_carrega_programa_na_memoria_secundaria(so_t *self, programa_t *programa)
{
  static int pos = 0;

  int end_ini = prog_end_carga(programa);
  int end_fim = end_ini + prog_tamanho(programa);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(programa, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }
  console_printf("carregado na memória física, %d-%d", end_ini, end_fim);
  return end_ini;
}

static int so_carrega_programa_na_memoria_fisica(so_t *self, programa_t *programa)
{
  int end_ini = prog_end_carga(programa);
  int end_fim = end_ini + prog_tamanho(programa);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(programa, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }
  console_printf("carregado na memória física, %d-%d", end_ini, end_fim);
  return end_ini;
}

static int so_carrega_programa_na_memoria_virtual(so_t *self,
                                                  programa_t *programa,
                                                  Process* processo)
{
  // t2: isto tá furado...
  // está simplesmente lendo para o próximo quadro que nunca foi ocupado,
  //   nem testa se tem memória disponível
  // com memória virtual, a forma mais simples de implementar a carga de um
  //   programa é carregá-lo para a memória secundária, e mapear todas as páginas
  //   da tabela de páginas do processo como inválidas. Assim, as páginas serão
  //   colocadas na memória principal por demanda. Para simplificar ainda mais, a
  //   memória secundária pode ser alocada da forma como a principal está sendo
  //   alocada aqui (sem reuso)
  int end_virt_ini = prog_end_carga(programa);
  int end_virt_fim = end_virt_ini + prog_tamanho(programa) - 1;
  int pagina_ini = end_virt_ini / TAM_PAGINA;
  int pagina_fim = end_virt_fim / TAM_PAGINA;
  int quadro_ini = self->quadro_livre;
  // mapeia as páginas nos quadros
  int quadro = quadro_ini;
  for (int pagina = pagina_ini; pagina <= pagina_fim; pagina++) {
    //tabpag_define_quadro(self->tabpag_global, pagina, quadro);
    quadro++;
  }
  self->quadro_livre = quadro;

  // carrega o programa na memória principal
  int end_fis_ini = quadro_ini * TAM_PAGINA;
  int end_fis = end_fis_ini;
  for (int end_virt = end_virt_ini; end_virt <= end_virt_fim; end_virt++) {
    if (mem_escreve(self->mem, end_fis, prog_dado(programa, end_virt)) != ERR_OK) {
      console_printf("Erro na carga da memória, end virt %d fís %d\n", end_virt,
                     end_fis);
      return -1;
    }
    end_fis++;
  }
  console_printf("carregado na memória virtual V%d-%d F%d-%d",
                 end_virt_ini, end_virt_fim, end_fis_ini, end_fis - 1);
  return end_virt_ini;
}

// ACESSO À MEMÓRIA DOS PROCESSOS {{{1

// copia uma string da memória do processo para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// O endereço é um endereço virtual de um processo.
// T2: Com memória virtual, cada valor do espaço de endereçamento do processo
//   pode estar em memória principal ou secundária (e tem que achar onde)
static bool so_copia_str_do_processo(so_t *self, int tam, char str[tam],
                                     int end_virt, Process* processo)
{
  if (!processo) return false;
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    // não tem memória virtual implementada, posso usar a mmu para traduzir
    //   os endereços e acessar a memória
    if (mmu_le(self->mmu, end_virt + indice_str, &caractere, usuario) != ERR_OK) {
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
