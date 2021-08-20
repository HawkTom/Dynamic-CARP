//
// Created by hao on 13/07/2020.
//

#ifndef DCARP_SIMULATOR_H
#define DCARP_SIMULATOR_H


#include "functions.h"

void dfs(int u, int *visited, int *parent, int *low, int *disc, int *time, int (* AdMatrix)[MAX_NODE_TAG_LENGTH]);
void findBridge(int (* AdMatrix)[MAX_NODE_TAG_LENGTH]);
void dynamicChange(Task *inst_tasks, Arc *inst_arcs, const int (*serve_tasks)[MAX_NODE_TAG_LENGTH], Vehicles *info, int *unserved_seq, unsigned int seed);
void executeSolution(Individual *Solution, int tau, Vehicles *state, int (*serve_tasks)[MAX_NODE_TAG_LENGTH], const Task *inst_tasks_vt);

typedef struct edge
{
    int head_node;
    int tail_node;
    int trav_cost;
    int demand;
    int link;
    unsigned int change;
    int unserved;
} Edge;


void saveGraph(Edge *graph, Vehicles info, int vnum, int new_req_edge_num, int new_nonreq_edge_num, int edge_num);

#endif //DCARP_SIMULATOR_H
