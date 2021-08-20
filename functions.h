//
// Created by hao on 01/06/2020.
//

#ifndef CARP_FUNCTIONS_H
#define CARP_FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "setjmp.h"

#define MAX_TASKS_TAG_LENGTH 500
#define MAX_ARCS_TAG_LENGTH 1001
#define MAX_NODE_TAG_LENGTH 300
#define INF 1000000000

#define MAX_TASK_SEG_LENGTH 550
#define MAX_ROUTE_TAG_LENGTH 50

#define MAX_TASK_TAG_LENGTH 500
#define MAX_TASK_SEQ_LENGTH 500

extern int terminal_condition; //if the best has nevere been changed over 20 iterations, stop the algorithm
extern int terminal_duration;

extern int req_arc_num;
extern int nonreq_arc_num;
extern int vertex_num;
extern int req_edge_num;
extern int nonreq_edge_num;
extern int vehicle_num;
extern int capacity;
extern int task_num;
extern int total_arc_num;
extern int DEPOT;
extern int trav_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
extern int serve_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];

extern int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
extern int shortest_path[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];

extern int cost_backup[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
extern int costlb;
extern int costub;
extern int demandlb;
extern int demandub;
extern char map[20];

extern double remain_cap_ratio_lb;
extern double remain_cap_ratio_ub;
extern int debug;

extern int edge_index[2];

extern jmp_buf buf;

typedef struct task
{
    int head_node;
    int tail_node;
    int dead_cost;
    int serv_cost;
    int demand;
    int inverse;
    int vt;
} Task;

// format: head ----> tail
typedef struct arc
{
    int head_node;
    int tail_node;
    int trav_cost;
    unsigned int change;
    int link;
} Arc;



typedef struct individual
{
    int Sequence[MAX_TASK_SEQ_LENGTH]; // task id with depot inside
    int Assignment[250]; // when the representation is chromosome,this is used as the chromosome.
    int TotalCost;
    int Loads[101];
    int TotalVioLoad;
    double Fitness;
//    int start[101]; // for DCARP
} Individual;


typedef struct vehicle_state
{
    int remain_seqs[MAX_TASK_SEQ_LENGTH];
    int stop[101];
    int remain_capacity[101];
} Vehicles;

void mod_dijkstra();

void readMap(Task *inst_tasks, Arc *inst_arcs, const char *path);

void update_cost(const Task *inst_tasks, const Arc *inst_arcs);

void construct_virtual_task(const Task *inst_tasks, Task *tasks_vt, const int *stop, const int *remain_capacity);

void nextScenario(Individual *Solution, Task *inst_tasks_vt, Task *inst_tasks, Arc *inst_arcs, Vehicles *state, unsigned int seed);




void inherit_solution(const int *prev_seq, int *new_seq, const Task *prev_inst_tasks, const Task *new_inst_tasks);
int repair_solution_greedy_insertion(Individual *solution, int *new_seq, const int *stop, const Task *inst_tasks_vt);
void repair_solution_with_new_vehicles(int *new_seq, Individual *solution, const int *stop, Vehicles *info, const Task *inst_tasks_vt);
int back_and_new(const int *stop, const Task *inst_tasks, const int seed);




int rand_choose(int num);

void rand_perm(int *a, int num);

void rand_selection(int *id1, int *id2, int popsize);

int max(int *Array);

void find_ele_positions(int *positions, int *a, int e);

void delete_element(int *a, int k);

void add_element(int *a, int e, int k);

int get_task_seq_total_cost(int *task_seq, const Task *inst_tasks);

int get_total_vio_load(int *route_seg_load);

int FindTask(int a, int b, const Task *ARPTask, int NO_Task);

void AssignArray(int *Array1, int *Array2);

void AssignSubArray(int *Array1, int k1, int k2, int *Array2);

void JoinArray(int *JointArray, int *Array);

int RandChoose(int n);

void ReverseDirection(int *Array, int k1, int k2);


// solver
void MAENS(const Task *inst_tasks, Individual *MAENSolution);
void MASDC(Individual *best_individual, Task *inst_tasks, const int *stop, const int *remain_capacity);
void MASDCvt(Individual *best_individual, Task *inst_tasks);
void TSA(const Task *inst_tasks, Individual *TSASolution);
void LMA(const Task *inst_tasks, Individual *LMASolution);


void TSAih(const Task *inst_tasks, Individual *TSASolution, Individual InitSolution);
void MASDCvtih(Individual *best_individual, Task *inst_tasks, Individual InitSolution);
void MAENSih(const Task *inst_tasks, Individual *MAENSolution, Individual InitSolution);
void LMAih(const Task *inst_tasks, Individual *LMASolution, Individual InitSolution);


void clear_solution(Individual *solution);

void check_tasks(const int *seq, int req_num, const Task *inst_tasks);
int check_load_num(int *seq);
void showSeq(int *seq);
int check_task_valid(int *seq);
int get_totoal_loads(int *seq, const Task *inst_tasks);
void get_each_load(int *seq, const Task *inst_tasks);
void check_solution_valid(Individual solution, const Task *inst_task);
void check_inhr(int *seq);
void check_seq_valid(Individual solution, const Task *inst_task);

#endif //CARP_FUNCTIONS_H
