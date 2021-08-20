//
// Created by hao on 27/07/2020.
//

#ifndef DCARP_MASDC_H
#define DCARP_MASDC_H

#include "../functions.h"
#include "../heutistic.h"

int MASDC_split(int *split_task_seq, int *one_task_seq, const int *stop, const int *remain_capacity, const Task *inst_tasks);
void roulette_wheel_selection(const int *fitness, int *choose);
int MASDC_local_search(Individual *indi, Individual *child, const int *stop, const int *remain_capacity, const Task *inst_tasks, const int max_fe);
void MASDC_crossover(const int *parent1, const int *parent2, int *child);
//void MASDC_crossover(int *parent1, int *parent2, int *child);


void rand_seq(Individual *rs_indi, const Task *inst_tasks, const int *serve_mark);
int same_or_not(const int *seq1, const int *seq2);



#endif //DCARP_MASDC_H
