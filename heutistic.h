//
// Created by hao on 03/06/2020.
//

#ifndef CARP_HEUTISTIC_H
#define CARP_HEUTISTIC_H

void path_scanning(Individual *ps_indi, const Task *inst_tasks, int *serve_mark);

void augment_merge(Individual *am_indi, const Task *inst_tasks);

void FredericksonHeuristic(int *FHRoute, int *Route, const Task *inst_tasks);

int split(int *split_task_seq, int *one_task_seq, const Task *inst_tasks);

int fleet_limited_split(int *split_task_seq, int *one_task_seq, const Task *inst_tasks);

#endif //CARP_HEUTISTIC_H
