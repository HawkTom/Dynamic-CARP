//
// Created by hao on 15/06/2020.
//

#ifndef CARP_MAENS_H
#define CARP_MAENS_H


#define MAX_POPSIZE 30
#define MAX_TOTALSIZE 210
#define MAX_NONDOMINATED_NUM 1000

#define M_trial 10
#define M_PROB 0.2
#define M_ite 100
#define M_wite 100
#define SI 1
#define DI 2
#define SWAP 3

#define MAX_NSIZE 10 // upper bound of nsize
#define MAX_ENSSIZE 100 // maximal ENS neighborhood size


# include "../functions.h"
# include "../heutistic.h"


typedef struct move
{
    int type;
    int task1;
    int task2;
    int orig_seg;
    int targ_seg;
    int orig_pos;
    int targ_pos;
    int total_cost;
    int total_vio_load;
    double fitness;
} Move;

void rand_scanning(Individual *rs_indi, const Task *inst_tasks, const int *serve_mark);
void indi_copy(Individual *target, Individual *source);
void SBX(Individual *xed_child, Individual *p1, Individual *p2, const Task *inst_tasks);
void lns_mut(Individual *c, Individual *p, Individual *best_fsb_solution, const Task *inst_tasks);

void lns(Individual *indi, double coef, int nsize, const Task *inst_tasks);

void single_insertion(Move *best_move, Individual *indi, double coef, const Task *inst_tasks);
void double_insertion(Move *best_move, Individual *indi, double coef, const Task *inst_tasks);
void swap(Move *best_move, Individual *indi, double coef, const Task *inst_tasks);


#endif //CARP_MAENS_H
