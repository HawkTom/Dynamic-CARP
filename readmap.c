//
// Created by hao on 09/06/2020.
//

#include "functions.h"

int costlb = INF;
int costub = 0;
int demandlb = INF;
int demandub = 0;

int edge_index[2];

void readMap(Task *inst_tasks, Arc *inst_arcs, const char *map1)
{
    FILE *fp;

    char dummy[101];

//    fp = fopen("../instances/example.dat", "r");
    char path[101];
    // strcpy(path, "../map/");
    strcpy(path, "");
    strcat(path, map1);
    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }

    while (fscanf(fp, "%s", dummy) != EOF)
    {

        if (strcmp(dummy, "VERTICES")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vertex_num);
        }
        else if (strcmp(dummy, "ARISTAS_REQ") == 0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &req_edge_num);
        }
        else if (strcmp(dummy, "ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &nonreq_edge_num);
        }
        else if (strcmp(dummy, "VEHICULOS")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vehicle_num);
        }
        else if (strcmp(dummy, "CAPACIDAD")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &capacity);
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_REQ")==0) {

            fscanf(fp, "%s", dummy);
            task_num = 2 * req_edge_num + req_arc_num;
            total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
            for (int i = 1; i <= req_edge_num; i++) {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_tasks[i].head_node);
                fscanf(fp, "%d)", &inst_tasks[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].serv_cost);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].demand);

                inst_tasks[i].dead_cost = inst_tasks[i].serv_cost;
                inst_tasks[i].inverse = i + req_edge_num;
                inst_tasks[i].vt = 0;

                inst_tasks[i + req_edge_num].head_node = inst_tasks[i].tail_node;
                inst_tasks[i + req_edge_num].tail_node = inst_tasks[i].head_node;
                inst_tasks[i + req_edge_num].dead_cost = inst_tasks[i].dead_cost;
                inst_tasks[i + req_edge_num].serv_cost = inst_tasks[i].serv_cost;
                inst_tasks[i + req_edge_num].demand = inst_tasks[i].demand;
                inst_tasks[i + req_edge_num].inverse = i;
                inst_tasks[i + req_edge_num].vt = 0;


                inst_arcs[i].head_node = inst_tasks[i].head_node;
                inst_arcs[i].tail_node = inst_tasks[i].tail_node;
                inst_arcs[i].trav_cost = inst_tasks[i].dead_cost;
                inst_arcs[i].change = 0;
                inst_arcs[i].link = 0;
                inst_arcs[i + req_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + req_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + req_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + req_edge_num].change = 0;
                inst_arcs[i+ req_edge_num].link = 0;

                if (costlb > inst_tasks[i].dead_cost)
                    costlb = inst_tasks[i].dead_cost;

                if (costub < inst_tasks[i].dead_cost)
                    costub = inst_tasks[i].dead_cost;

                if (demandlb > inst_tasks[i].demand)
                    demandlb = inst_tasks[i].demand;

                if (demandub < inst_tasks[i].demand)
                    demandub = inst_tasks[i].demand;

            }
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i=task_num+1; i<=task_num+nonreq_edge_num;i++)
            {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_arcs[i].head_node);
                fscanf(fp, "%d)", &inst_arcs[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_arcs[i].trav_cost);
                inst_arcs[i].change = 0;
                inst_arcs[i].link = 0;

                inst_arcs[i + nonreq_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + nonreq_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + nonreq_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + nonreq_edge_num].change = 0;
                inst_arcs[i + nonreq_edge_num].link = 0;

                if (costlb > inst_arcs[i].trav_cost)
                    costlb = inst_arcs[i].trav_cost;

                if (costub < inst_arcs[i].trav_cost)
                    costub = inst_arcs[i].trav_cost;

            }
        }
        else if (strcmp(dummy, "DEPOSITO")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &DEPOT);
        }

    }

    fclose(fp);

    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;

    for (int i=1; i<=total_arc_num; i++)
    {
        cost_backup[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }
    memset(edge_index, 0, sizeof(edge_index));
    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;
//    edge_index[2] = task_num + nonreq_edge_num;
}