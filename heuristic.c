#include <iso646.h>
//
// Created by hao on 03/06/2020.
//

#include "functions.h"


void GetConnectivePiece(int Root, int CurrentPiece, int *PieceMark, int (*Neighbors)[MAX_NODE_TAG_LENGTH]);

void GetMinCostTree(int (*CPMCTree)[MAX_NODE_TAG_LENGTH], int (*CPMinCost)[MAX_NODE_TAG_LENGTH]);

void GetEulerRoute(int *EulerRoute, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes);

void FindCircuit(int *EulerRoute, int CurrentNode, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes);

void EvenAllOddNodes(int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *OddNodes);

void route_process(int *split_task_seq, const int *process_split, const Task *inst_tasks);
void indi_route_converter(Individual *dst, Individual *src, const Task *inst_tasks);


int split(int *split_task_seq, int *one_task_seq, const Task *inst_tasks)
{
    int V[MAX_TASK_TAG_LENGTH], P[MAX_TASK_TAG_LENGTH];
    V[0] = 0;
    P[0] = 0;
    int i, j;

    for(i = 1; i <= one_task_seq[0]; i++)
    {
        V[i] = INF;
    }

    for(i = 1; i <= one_task_seq[0]; i++)
    {
        int load = 0, cost = 0;
        j = i;
        while (j <= one_task_seq[0] && load <= capacity)
        {
            // for dynamic CARP, once a virtual task being added, there is a new route.
            // the load reset to 0
            if (inst_tasks[one_task_seq[j]].vt > 0)
            {
                load = 0;
            }
            // for static CARP, each split represented one route
            load += inst_tasks[one_task_seq[j]].demand;
            if (j == i)
            {
                cost = min_cost[DEPOT][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT];
            } else {
                cost += min_cost[inst_tasks[one_task_seq[j-1]].tail_node][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT] \
                        - min_cost[inst_tasks[one_task_seq[j-1]].tail_node][DEPOT];
            }
            if (load <= capacity)
            {
                int V_new = V[i-1] + cost;
                if (V_new < V[j])
                {
                    V[j] = V_new;
                    P[j] = i-1;
                }
                j ++;
            }
        }
    }

    int process_split[MAX_TASK_SEQ_LENGTH];
    memset(process_split, 0, sizeof(process_split));

    process_split[0] = 1;
    process_split[1] = 0;

    j = one_task_seq[0];
    int ptr = P[j];
    while (ptr > 0)
    {
        for (int k=ptr+1; k<=j; k++)
        {
            process_split[0] ++;
            process_split[process_split[0]] = one_task_seq[k];
        }

        process_split[0] ++;
        process_split[process_split[0]] = 0;

        j = ptr;
        ptr = P[j];
    }

    for (int k=1; k <= j; k++)
    {
        process_split[0] ++;
        process_split[process_split[0]] = one_task_seq[k];
    }
    process_split[0] ++;
    process_split[process_split[0]] = 0;

    route_process(split_task_seq, process_split, inst_tasks);

    int opt_cost = V[one_task_seq[0]];
    return opt_cost;

}

void route_process(int *split_task_seq, const int *process_split, const Task *inst_tasks)
{
    int i;
    split_task_seq[0] = 0;
    for (i = 1; i <= process_split[0]; i++)
    {
        if (inst_tasks[process_split[i]].vt > 0 && process_split[i-1] != 0)
        {
            split_task_seq[0] ++;
            split_task_seq[split_task_seq[0]] = 0;
        }
        split_task_seq[0] ++;
        split_task_seq[split_task_seq[0]] = process_split[i];
    }
}

int fleet_limited_split(int *split_task_seq, int *one_task_seq, const Task *inst_tasks)
{
    int aux_cost[MAX_TASK_SEG_LENGTH][MAX_TASK_TAG_LENGTH];
    int opt_cost[MAX_TASK_SEG_LENGTH][MAX_ROUTE_TAG_LENGTH];
    int pred_task[MAX_TASK_SEG_LENGTH][MAX_ROUTE_TAG_LENGTH];
    int tmp_task_seq[MAX_TASK_SEG_LENGTH];


    int i, j, k;
    for (i = 1; i <= one_task_seq[0]; i++)
    {
        for (j = 0; j < one_task_seq[0]; ++j)
        {
            aux_cost[i][j] = INF;
        }
    }

    for (i = 0; i<=one_task_seq[0]; i++)
    {
        for (j = 0; j<=vehicle_num; j++)
        {
            opt_cost[i][j] = INF;
        }
    }
    opt_cost[0][0] = 0;

    for (i = 0; i<=one_task_seq[0]; i++)
    {
        int load = 0;
        int cost;
        j = i;
        while (j <= one_task_seq[0])
        {
            load += inst_tasks[one_task_seq[j]].demand;
            if (load > capacity)
                break;

            if (j == i)
            {
                cost = min_cost[DEPOT][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT];
            }
            else
            {
                cost += min_cost[inst_tasks[one_task_seq[j-1]].tail_node][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT] \
                        - min_cost[inst_tasks[one_task_seq[j-1]].tail_node][DEPOT];
            }
            aux_cost[i][j] = cost;
            j ++;
        }
    }

    // dynamic programming
    for (i=1; i<=one_task_seq[0]; i++)
    {
        for (j=1; j <= one_task_seq[0]; j++)
        {
            for (k =0; k < j; k++)
            {
                if (opt_cost[k][i-1] < INF && opt_cost[k][i-1] + aux_cost[k+1][j] < opt_cost[j][i])
                {
                    opt_cost[j][i] = opt_cost[k][i-1] + aux_cost[k+1][j];
                    pred_task[j][i] = k;
                }
            }
        }
    }

    int all_opt_num;
    int all_opt_cost = INF;
    for (i=1; i<=vehicle_num; i++)
    {
        if (opt_cost[one_task_seq[0]][i] < all_opt_cost)
        {
            all_opt_cost = opt_cost[one_task_seq[0]][i];
            all_opt_num = i;
        }
    }

    if (all_opt_cost == INF)
    {
        split_task_seq[0] = 0;
        return all_opt_cost;
    }

    tmp_task_seq[0] = 1;
    tmp_task_seq[tmp_task_seq[0]] = one_task_seq[0];

    while (1)
    {
        tmp_task_seq[0] ++;
        tmp_task_seq[tmp_task_seq[0]] = pred_task[tmp_task_seq[tmp_task_seq[0] - 1]][all_opt_num];
        all_opt_num --;

        if (tmp_task_seq[tmp_task_seq[0]] == 0)
            break;
    }

    split_task_seq[0] = 1;
    split_task_seq[1] = 0;

    for (i= tmp_task_seq[0]; i >1; i--)
    {
        for (j = tmp_task_seq[i]+1; j<=tmp_task_seq[i-1]; j++)
        {
            split_task_seq[0]++;
            split_task_seq[split_task_seq[0]] = one_task_seq[j];
        }
        split_task_seq[0]++;
        split_task_seq[split_task_seq[0]] = 0;
    }

    return  all_opt_cost;

}

void augment_merge(Individual *am_indi, const Task *inst_tasks)
{
    // no augment phase;
    int i, j, s;
    int Route[req_edge_num + 1][MAX_TASK_SEG_LENGTH];
    int Endnodes[req_edge_num + 1][2];
    int load[req_edge_num + 1];
    int task_position[2 * req_edge_num + 1];
    int trip_pair[2 * req_edge_num * (req_edge_num - 1) + 1][2];
    int cost_saving[2 * req_edge_num * (req_edge_num - 1) + 1];


    memset(Route, 0, sizeof(Route));
    memset(trip_pair, 0, sizeof(trip_pair));
    memset(cost_saving, 0, sizeof(cost_saving));
    memset(task_position, 0, sizeof(task_position));
    memset(load, 0, sizeof(load));
    memset(Endnodes, 0, sizeof(Endnodes));

    Route[0][0] = 0;
    load[0] = 0;
    for (i = 1; i <= req_edge_num; i++)
    {
        Route[0][0] ++;
        Route[Route[0][0]][0] = 1;
        Route[Route[0][0]][1] = i;
        Endnodes[Route[0][0]][0] = i;
        Endnodes[Route[0][0]][1] = i + req_edge_num;
        load[0] ++;
        load[load[0]] = inst_tasks[i].demand;
        task_position[i] = i;
        task_position[i+req_edge_num] = i;
    }

    trip_pair[0][0] = 0;
    cost_saving[0] = 0;
    int inv_i, inv_j;
    for (i = 1; i < req_edge_num; i++)
    {

        inv_i = inst_tasks[i].inverse;
        for (j = i + 1; j <= req_edge_num; j++)
        {

            inv_j = inst_tasks[j].inverse;
            s = min_cost[DEPOT][inst_tasks[i].tail_node] + min_cost[inst_tasks[j].head_node][DEPOT] - min_cost[inst_tasks[i].tail_node][inst_tasks[j].head_node];
            trip_pair[0][0] += 1;
            trip_pair[trip_pair[0][0]][0] = i;
            trip_pair[trip_pair[0][0]][1] = j;
            cost_saving[0]++;
            cost_saving[cost_saving[0]] = s;


            s = min_cost[DEPOT][inst_tasks[i].tail_node] + min_cost[inst_tasks[inv_j].head_node][DEPOT] - min_cost[inst_tasks[i].tail_node][inst_tasks[inv_j].head_node];
            trip_pair[0][0] += 1;
            trip_pair[trip_pair[0][0]][0] = i;
            trip_pair[trip_pair[0][0]][1] = inv_j;
            cost_saving[0]++;
            cost_saving[cost_saving[0]] = s;


            s = min_cost[DEPOT][inst_tasks[inv_i].tail_node] + min_cost[inst_tasks[j].head_node][DEPOT] - min_cost[inst_tasks[inv_i].tail_node][inst_tasks[j].head_node];
            trip_pair[0][0] += 1;
            trip_pair[trip_pair[0][0]][0] = inv_i;
            trip_pair[trip_pair[0][0]][1] = j;
            cost_saving[0]++;
            cost_saving[cost_saving[0]] = s;


            s = min_cost[DEPOT][inst_tasks[inv_i].tail_node] + min_cost[inst_tasks[inv_j].head_node][DEPOT] - min_cost[inst_tasks[inv_i].tail_node][inst_tasks[inv_j].head_node];
            trip_pair[0][0] += 1;
            trip_pair[trip_pair[0][0]][0] = inv_i;
            trip_pair[trip_pair[0][0]][1] = inv_j;
            cost_saving[0]++;
            cost_saving[cost_saving[0]] = s;
        }

    }

    // sort in decreasing order;
    int index[cost_saving[0]+1];
    index[0] = cost_saving[0];
    for (i = 1; i <= cost_saving[0]; i++) {
        index[i] = i;
    }

    int temp;
    for (i = 1; i < cost_saving[0]; i++) {
        for (j = i + 1; j <= cost_saving[0]; j++) {
            if (cost_saving[i] < cost_saving[j]) {
                temp = cost_saving[i];
                cost_saving[i] = cost_saving[j];
                cost_saving[j] = temp;

                temp = index[i];
                index[i] = index[j];
                index[j] = temp;
            }
        }
    }

    // merge, head ----> tail
    int node1, node2;
    int node_active[2 * req_edge_num + 1];
    for (i = 1; i <= 2*req_edge_num; i++)
    {
        if (i <= req_edge_num){
            node_active[i] = 1; // head
        } else{
            node_active[i] = 2; // tail
        }
    }


    int k, p;
    int newhead, newtail;
    for (i = 1; i <= trip_pair[0][0]; i++)
    {
        node1 = trip_pair[index[i]][0];
        node2 = trip_pair[index[i]][1];

        if (node_active[node1] && node_active[node2])
        {
            k = task_position[node1];
            p = task_position[node2];
            if ( k != p && load[k] + load[p] <= capacity)
            {
                if (node_active[node1] == 1)
                {
                    newhead = Endnodes[k][0];
                    newtail = Endnodes[p][1];
                    for (j = 1; j <= Route[k][0]; j++)
                    {
                        Route[k][j] = inst_tasks[Route[k][j]].inverse;
                    }
                    ReverseDirection(Route[k], 1, Route[k][0]);
                    newhead = Endnodes[k][1];
                }

                if (node_active[node2] == 2)
                {
                    for (j = 1; j <= Route[p][0]; j++)
                    {
                        Route[p][j] = inst_tasks[Route[p][j]].inverse;
                    }
                    ReverseDirection(Route[p], 1, Route[p][0]);
                    newtail = Endnodes[p][0];
                }
                JoinArray(Route[k], Route[p]);


                load[k] += load[p];
                load[p] = 0;

                Route[p][0] = 0;
                Endnodes[k][0] = newhead;
                Endnodes[k][1] = newtail;

                node_active[node1] = 0;
                node_active[node2] = 0;
                node_active[newhead] = 1;
                node_active[newtail] = 2;

                task_position[inst_tasks[node1].inverse] = k;
                task_position[inst_tasks[node2].inverse] = k;

            }

        }
    }

    am_indi->Sequence[0] = 1;
    am_indi->Sequence[1] = 0;
    am_indi->Loads[0] = 0;

    for (i = 1; i <= Route[0][0]; i++)
    {
        if (Route[i][0] > 0)
        {
            JoinArray(am_indi->Sequence, Route[i]);
            am_indi->Loads[0] ++;
            am_indi->Loads[am_indi->Loads[0]] = load[i];
            am_indi->Sequence[0] ++;
            am_indi->Sequence[am_indi->Sequence[0]] = 0;
        }
    }

    am_indi->TotalCost = get_task_seq_total_cost(am_indi->Sequence, inst_tasks);
    am_indi->TotalVioLoad = 0;

}

void path_scanning(Individual *ps_indi, const Task *inst_tasks, const int *serve_mark)
{
    // min_cost, NRE, NRA, NVeh, capacity, is the extern variables.
    int i, j, k;
    int serve_task_num=0;
    for (i=req_edge_num+1; i<=task_num; i++)
    {
        if (serve_mark[i])
        {
            serve_task_num ++;
        }
    }
    int load, trial, mindist;
    int unserved_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH], nearest_task[MAX_TASK_TAG_LENGTH];
    int nearest_isol_task[MAX_TASK_TAG_LENGTH], nearest_inci_task[MAX_TASK_TAG_LENGTH], sel_task[MAX_TASK_TAG_LENGTH];
    int current_task, next_task;

    int positions[MAX_TASK_SEQ_LENGTH];

    ps_indi->TotalCost = INF;
    Individual tmp_indi1, tmp_indi2, tmp_indi3, tmp_indi4, tmp_indi5;

    int dep_dist[MAX_TASK_TAG_LENGTH], max_dep_dist, min_dep_dist;
    double yield[MAX_TASK_TAG_LENGTH], max_yield, min_yield;

    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        if(inst_tasks[i].demand > capacity)
        {
            printf("error. \n");
            // longjmp(buf, 2);
            exit(0);
        }

        dep_dist[i] = min_cost[inst_tasks[i].tail_node][DEPOT];
        yield[i] = 1.0*inst_tasks[i].demand/inst_tasks[i].serv_cost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 1 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi1.Sequence[0] = 1;
    tmp_indi1.Sequence[1] = 0;
    tmp_indi1.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    // printf("start: %d %d\n", tmp_indi1.Sequence[0], tmp_indi1.Loads[0]);
    while(trial < serve_task_num)
    {
        // if (tmp_indi1.Sequence[0] > 1000)
        // {
        //     printf("error");
        // }
        // printf("%d %d \t", tmp_indi1.Sequence[0], tmp_indi1.Loads[0]);
        current_task = tmp_indi1.Sequence[tmp_indi1.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;
        // printf("%d \n", current_task);
        // if (tmp_indi1.Sequence[0] == 52 && tmp_indi1.Loads[0] == 9 && current_task == 76)
        // {
        //     printf("error");
        // }

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi1.Sequence[0] ++;
            tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
            tmp_indi1.Loads[0] ++;
            tmp_indi1.Loads[tmp_indi1.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        max_dep_dist = -1;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (dep_dist[nearest_isol_task[i]] > max_dep_dist)
            {
                max_dep_dist = dep_dist[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (dep_dist[nearest_isol_task[i]] == max_dep_dist)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }
        k = 1;
        next_task = sel_task[k];

        trial ++;
        // printf("%d %d\n", tmp_indi1.Sequence[0], tmp_indi1.Loads[0]);
        tmp_indi1.Sequence[0]++;
        tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = next_task;
        // if (tmp_indi1.Sequence[0] > 1000)
        // {
        //     printf("error");
        // }
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
//        printf("serve_task_num: %d\n", serve_task_num);
    }

//    printf("serve_task_num: %d\n", serve_task_num);
    tmp_indi1.Sequence[0] ++    ;
    tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
    tmp_indi1.Loads[0] ++;
    tmp_indi1.Loads[tmp_indi1.Loads[0]] = load;

    tmp_indi1.TotalCost = get_task_seq_total_cost(tmp_indi1.Sequence, inst_tasks);
//    tmp_indi1.TotalVioLoad = get_total_vio_load(tmp_indi1.Loads);

    if (tmp_indi1.TotalCost < ps_indi->TotalCost)
    {
//        memcpy(ps_indi->Sequence, tmp_indi1.Sequence, (tmp_indi1.Sequence[0]+1)*sizeof(int));
//        memcpy(ps_indi->Loads, tmp_indi1.Loads, (tmp_indi1.Loads[0]+1)*sizeof(int));
        indi_route_converter(ps_indi, &tmp_indi1, inst_tasks);
        ps_indi->TotalCost = tmp_indi1.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 2 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi2.Sequence[0] = 1;
    tmp_indi2.Sequence[1] = 0;
    tmp_indi2.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi2.Sequence[tmp_indi2.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi2.Sequence[0] ++;
            tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
            tmp_indi2.Loads[0] ++;
            tmp_indi2.Loads[tmp_indi2.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        min_dep_dist = INF;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (dep_dist[nearest_isol_task[i]] < min_dep_dist)
            {
                min_dep_dist = dep_dist[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (dep_dist[nearest_isol_task[i]] == min_dep_dist)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi2.Sequence[0]++;
        tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi2.Sequence[0] ++ ;
    tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
    tmp_indi2.Loads[0] ++;
    tmp_indi2.Loads[tmp_indi2.Loads[0]] = load;

    tmp_indi2.TotalCost = get_task_seq_total_cost(tmp_indi2.Sequence, inst_tasks);
//    tmp_indi2.TotalVioLoad = get_total_vio_load(tmp_indi2.Loads);

    if (tmp_indi2.TotalCost < ps_indi->TotalCost)
    {
//        memcpy(ps_indi->Sequence, tmp_indi2.Sequence, (tmp_indi2.Sequence[0]+1)*sizeof(int));
//        memcpy(ps_indi->Loads, tmp_indi2.Loads, (tmp_indi2.Loads[0]+1)*sizeof(int));
        indi_route_converter(ps_indi, &tmp_indi2, inst_tasks);
        ps_indi->TotalCost = tmp_indi2.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 3 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi3.Sequence[0] = 1;
    tmp_indi3.Sequence[1] = 0;
    tmp_indi3.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi3.Sequence[tmp_indi3.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi3.Sequence[0] ++;
            tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
            tmp_indi3.Loads[0] ++;
            tmp_indi3.Loads[tmp_indi3.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        max_yield = -1;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (yield[nearest_isol_task[i]] > max_yield)
            {
                max_yield = yield[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (yield[nearest_isol_task[i]] == max_yield)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi3.Sequence[0]++;
        tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;
        // printf("nt:%d, ntIv: %d \n", next_task, inst_tasks[next_task].inverse);
        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi3.Sequence[0] ++ ;
    tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
    tmp_indi3.Loads[0] ++;
    tmp_indi3.Loads[tmp_indi3.Loads[0]] = load;

    tmp_indi3.TotalCost = get_task_seq_total_cost(tmp_indi3.Sequence, inst_tasks);
//    tmp_indi3.TotalVioLoad = get_total_vio_load(tmp_indi3.Loads);

    if (tmp_indi3.TotalCost < ps_indi->TotalCost)
    {
//        memcpy(ps_indi->Sequence, tmp_indi3.Sequence, (tmp_indi3.Sequence[0]+1)*sizeof(int));
//        memcpy(ps_indi->Loads, tmp_indi3.Loads, (tmp_indi3.Loads[0]+1)*sizeof(int));
        indi_route_converter(ps_indi, &tmp_indi3, inst_tasks);
        ps_indi->TotalCost = tmp_indi3.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 4 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi4.Sequence[0] = 1;
    tmp_indi4.Sequence[1] = 0;
    tmp_indi4.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi4.Sequence[tmp_indi4.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi4.Sequence[0] ++;
            tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
            tmp_indi4.Loads[0] ++;
            tmp_indi4.Loads[tmp_indi4.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        min_yield = INF;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (yield[nearest_isol_task[i]] < min_yield)
            {
                min_yield = yield[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (yield[nearest_isol_task[i]] == min_yield)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi4.Sequence[0]++;
        tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;
//        printf("%d %d \n", next_task, inst_tasks[next_task].inverse);
//        if (next_task == 70)
//        {
//            printf("error");
//        }
        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi4.Sequence[0] ++ ;
    tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
    tmp_indi4.Loads[0] ++;
    tmp_indi4.Loads[tmp_indi4.Loads[0]] = load;

    tmp_indi4.TotalCost = get_task_seq_total_cost(tmp_indi4.Sequence, inst_tasks);
//    tmp_indi4.TotalVioLoad = get_total_vio_load(tmp_indi4.Loads);

    if (tmp_indi4.TotalCost < ps_indi->TotalCost)
    {
//        memcpy(ps_indi->Sequence, tmp_indi4.Sequence, (tmp_indi4.Sequence[0]+1)*sizeof(int));
//        memcpy(ps_indi->Loads, tmp_indi4.Loads, (tmp_indi4.Loads[0]+1)*sizeof(int));
        indi_route_converter(ps_indi, &tmp_indi4, inst_tasks);
        ps_indi->TotalCost = tmp_indi4.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 5 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi5.Sequence[0] = 1;
    tmp_indi5.Sequence[1] = 0;
    tmp_indi5.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi5.Sequence[tmp_indi5.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi5.Sequence[0] ++;
            tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
            tmp_indi5.Loads[0] ++;
            tmp_indi5.Loads[tmp_indi5.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        if (load < capacity/2)
        {
            max_dep_dist = -1;
            sel_task[0] = 0;
            for (i = 1; i <= nearest_isol_task[0]; i++)
            {
                if (dep_dist[nearest_isol_task[i]] > max_dep_dist)
                {
                    max_dep_dist = dep_dist[nearest_isol_task[i]];
                    sel_task[0] = 1;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
                else if (dep_dist[nearest_isol_task[i]] == max_dep_dist)
                {
                    sel_task[0] ++;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
            }
        }
        else
        {
            min_dep_dist = INF;
            sel_task[0] = 0;
            for (i = 1; i <= nearest_isol_task[0]; i++)
            {
                if (dep_dist[nearest_isol_task[i]] < min_dep_dist)
                {
                    min_dep_dist = dep_dist[nearest_isol_task[i]];
                    sel_task[0] = 1;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
                else if (dep_dist[nearest_isol_task[i]] == min_dep_dist)
                {
                    sel_task[0] ++;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi5.Sequence[0]++;
        tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi5.Sequence[0] ++ ;
    tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
    tmp_indi5.Loads[0] ++;
    tmp_indi5.Loads[tmp_indi5.Loads[0]] = load;

    tmp_indi5.TotalCost = get_task_seq_total_cost(tmp_indi5.Sequence, inst_tasks);

    if (tmp_indi5.TotalCost < ps_indi->TotalCost)
    {
        indi_route_converter(ps_indi, &tmp_indi5, inst_tasks);
        ps_indi->TotalCost = tmp_indi5.TotalCost;
    }

    ps_indi->TotalVioLoad = 0;

}

void indi_route_converter(Individual *dst, Individual *src, const Task *inst_tasks)
{
    int i, load;
    load = 0;
    memset(dst->Sequence, 0, sizeof(int) * MAX_TASK_SEQ_LENGTH);
    memset(dst->Loads, 0, sizeof(int) * 50);
    dst->Sequence[0] = 1;
    dst->Sequence[1] = 0;
    for (i = 2; i <= src->Sequence[0]; i++)
    {
        if (src->Sequence[i] == 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
            continue;
        }
        if (inst_tasks[src->Sequence[i]].vt > 0 && src->Sequence[i-1] != 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
        }

        load += inst_tasks[src->Sequence[i]].demand;
        dst->Sequence[0] ++;
        dst->Sequence[dst->Sequence[0]] = src->Sequence[i];
    }
}

void FredericksonHeuristic(int *FHRoute, int *Route, const Task *inst_tasks)
{
    // min_cost, task_num, vertex_num are extern variables
    int i, j, k, u, v;

    int EulerRoute[3*(Route[0]-1)];

    int Degree[MAX_NODE_TAG_LENGTH];
    int OddNodes[MAX_NODE_TAG_LENGTH];

    int Nodes[2*Route[0]+1];
    Nodes[0] = 1;
    Nodes[1] = 1;

    int flag[MAX_NODE_TAG_LENGTH];
    memset(flag, 0, sizeof(flag));
    flag[1] = 1;

    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    int TaskMatrix[MAX_TASK_SEQ_LENGTH][MAX_TASK_SEQ_LENGTH];

    memset(AdMatrix, 0, sizeof(AdMatrix));
    memset(TaskMatrix, 0, sizeof(TaskMatrix));

    int Neighbors[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i=1; i<=vertex_num; i++)
    {
        Neighbors[i][0] = 0;
    }

//    int ConnectivePieceNodes[2*Route[0]+2][2*Route[0]+2];
    int ConnectivePieceNodes[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(ConnectivePieceNodes, 0, sizeof(ConnectivePieceNodes));
    ConnectivePieceNodes[0][0] = 0;

    int PieceMark[MAX_NODE_TAG_LENGTH];
    memset(PieceMark, 0, sizeof(PieceMark));
    PieceMark[0] = vertex_num;

    int CPMinCost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i = 1; i<=vertex_num; i++)
    {
        for(j = 1; j<=vertex_num; j++)
        {
            CPMinCost[i][j] = INF;
        }
    }

    int CPMCTree[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    int CurrentPiece = 0;

    int virtualTask[101];
    memset(virtualTask, 0, sizeof(virtualTask));

    for (i = 2; i < Route[0]; i++)
    {
        // ignore the virtual task
        if (inst_tasks[Route[i]].vt)
        {
            virtualTask[0] ++;
            virtualTask[virtualTask[0]] = Route[i];
            continue;
        }
        Neighbors[inst_tasks[Route[i]].tail_node][0]++;
        Neighbors[inst_tasks[Route[i]].tail_node][Neighbors[inst_tasks[Route[i]].tail_node][0]] = inst_tasks[Route[i]].head_node;
        Neighbors[inst_tasks[Route[i]].head_node][0]++;
        Neighbors[inst_tasks[Route[i]].head_node][Neighbors[inst_tasks[Route[i]].head_node][0]] = inst_tasks[Route[i]].tail_node;

        if (! flag[inst_tasks[Route[i]].tail_node])
        {
            Nodes[0] ++;
            Nodes[Nodes[0]] = inst_tasks[Route[i]].tail_node;
            flag[inst_tasks[Route[i]].tail_node] = 1;
        }
        if (! flag[inst_tasks[Route[i]].head_node])
        {
            Nodes[0] ++;
            Nodes[Nodes[0]] = inst_tasks[Route[i]].head_node;
            flag[inst_tasks[Route[i]].head_node] = 1;
        }

        AdMatrix[inst_tasks[Route[i]].head_node][inst_tasks[Route[i]].tail_node] ++;
        AdMatrix[inst_tasks[Route[i]].tail_node][inst_tasks[Route[i]].head_node] ++;
        TaskMatrix[inst_tasks[Route[i]].head_node][inst_tasks[Route[i]].tail_node] ++;
        TaskMatrix[inst_tasks[Route[i]].tail_node][inst_tasks[Route[i]].head_node] ++;
    }

    for (i=1; i<=Nodes[0]; i++)
    {
        if (PieceMark[Nodes[i]] > 0)
            continue;
        CurrentPiece ++;
        GetConnectivePiece(Nodes[i], CurrentPiece, PieceMark, Neighbors);
    }

    ConnectivePieceNodes[0][0] = max(PieceMark);
    for(i=1; i<=Nodes[0]; i++)
    {
        ConnectivePieceNodes[PieceMark[Nodes[i]]][0] ++;
        ConnectivePieceNodes[PieceMark[Nodes[i]]][ConnectivePieceNodes[PieceMark[Nodes[i]]][0]] = Nodes[i];
    }
    if (ConnectivePieceNodes[0][0] > 1)
    {
        CPMinCost[0][0] = ConnectivePieceNodes[0][0];
        CPMCTree[0][0] = ConnectivePieceNodes[0][0];

        int BNode[CPMinCost[0][0]+1][CPMinCost[0][0]+1];
        int ENode[CPMinCost[0][0]+1][CPMinCost[0][0]+1];

        for (i=1; i < ConnectivePieceNodes[0][0]; i++)
        {
            for (j=i+1; j <= ConnectivePieceNodes[0][0]; j++)
            {
                if (i==j)
                    continue;
                for(u = 1; u <= ConnectivePieceNodes[i][0]; u++)
                {
                    for(v = 1; v <= ConnectivePieceNodes[j][0]; v++)
                    {
                        if(min_cost[ConnectivePieceNodes[i][u]][ConnectivePieceNodes[j][v]] < CPMinCost[i][j])
                        {
                            CPMinCost[i][j] = min_cost[ConnectivePieceNodes[i][u]][ConnectivePieceNodes[j][v]];
                            CPMinCost[j][i] = CPMinCost[i][j];
                            BNode[i][j] = ConnectivePieceNodes[i][u];
                            ENode[i][j] = ConnectivePieceNodes[j][v];

                            BNode[j][i] = ENode[i][j];
                            ENode[j][i] = BNode[i][j];
                        }
                    }
                }
            }
        }

        GetMinCostTree(CPMCTree, CPMinCost);

        for (i=1;  i< CPMCTree[0][0]; i++)
        {
            for (j=i+1; j <= CPMCTree[0][0]; j++)
            {
                if (CPMCTree[i][j])
                {
                    AdMatrix[BNode[i][j]][ENode[i][j]] ++;
                    AdMatrix[ENode[i][j]][BNode[i][j]] ++;

                    if (CPMinCost[i][j] == INF)
                    {
                        CPMinCost[i][j] = min_cost[BNode[i][j]][ENode[i][j]];
                        CPMinCost[j][i] = CPMinCost[i][j];
                    }
                }
            }
        }
    }
    memset(Degree, 0, sizeof(Degree));
    OddNodes[0] = 0;
    for (i = 1; i <= Nodes[0]; i++)
    {
        for (j = 1; j <= Nodes[0]; j++ )
        {
            Degree[Nodes[i]] += AdMatrix[Nodes[i]][Nodes[j]];
        }
        if (Degree[Nodes[i]] % 2 == 1)
        {
            OddNodes[0] ++;
            OddNodes[OddNodes[0]] = Nodes[i];
        }
    }

    // EvenAllNodes
    EvenAllOddNodes(AdMatrix, OddNodes);

    // GetEulerRoute
    GetEulerRoute(EulerRoute, AdMatrix, Nodes);


    FHRoute[0] = 1;
    FHRoute[1] = 0;

    for (i = 1; i < EulerRoute[0]; i++)
    {
        if (TaskMatrix[EulerRoute[i]][EulerRoute[i+1]])
        {
            TaskMatrix[EulerRoute[i]][EulerRoute[i+1]] --;
            TaskMatrix[EulerRoute[i+1]][EulerRoute[i]] --;
            FHRoute[0] ++;
            FHRoute[FHRoute[0]] = FindTask(EulerRoute[i], EulerRoute[i+1], inst_tasks, task_num);
        }
    }

    FHRoute[0] ++;
    FHRoute[FHRoute[0]] = 0;

    // ignore the virtual task, and insert them in the position where the outside vehicle is closest.
    if (virtualTask[0] > 0)
    {
        int insert_flag[FHRoute[0]];
        memset(insert_flag, 0, sizeof(insert_flag));
        int tmp;
        for (i = 1; i < virtualTask[0]; i++)
        {
            for (j = i+1; j <= virtualTask[0]; j++)
            {
                if (inst_tasks[virtualTask[i]].serv_cost < inst_tasks[virtualTask[j]].serv_cost)
                {
                    tmp = virtualTask[j];
                    virtualTask[j] = virtualTask[i];
                    virtualTask[i] = tmp;
                }
            }
        }

        for (i = 1; i <= virtualTask[0]; i++)
        {
            int currTask = virtualTask[i];
            int min_reduce_cost = INF, reduce_cost, insert_position;
            for (j = 1; j < FHRoute[0]; j++)
            {
                reduce_cost = min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[virtualTask[i]].head_node]
                 + min_cost[inst_tasks[virtualTask[i]].tail_node][inst_tasks[FHRoute[j+1]].head_node]
                 - min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[FHRoute[j+1]].head_node];
                if (reduce_cost < min_reduce_cost)
                {
                    min_reduce_cost = reduce_cost;
                    insert_position = j+1;
                }
            }
            add_element(FHRoute, virtualTask[i], insert_position);
        }
    }

}




void GetConnectivePiece(int Root, int CurrentPiece, int *PieceMark, int (*Neighbors)[MAX_NODE_TAG_LENGTH])
{
    int i, j;

    PieceMark[Root] = CurrentPiece;

    for (i = 1; i <= Neighbors[Root][0]; i++)
    {
        if (PieceMark[Neighbors[Root][i]] > 0)
            continue;

        GetConnectivePiece(Neighbors[Root][i], CurrentPiece, PieceMark, Neighbors);
    }
}

void GetMinCostTree( int (*CPMCTree)[MAX_NODE_TAG_LENGTH], int (*CPMinCost)[MAX_NODE_TAG_LENGTH])
{
    int i, j, k;
    int u, v;
    int MCost;

    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            CPMCTree[i][j] = 0;
        }
    }

    int ColoredArcs[CPMinCost[0][0]+1][CPMinCost[0][0]+1];
    int Visited[CPMinCost[0][0]+1];

    memset(ColoredArcs, 0, sizeof(ColoredArcs));
    memset(Visited, 0, sizeof(Visited));

    Visited[1] = 1;

    for (i = 1; i <= CPMinCost[0][0]; i++)
    {
        if (CPMinCost[1][i] < INF)
        {
            ColoredArcs[1][i] = 1;
        }

        if (CPMinCost[i][1] < INF)
        {
            ColoredArcs[i][1] = 1;
        }
    }

    for (k = 1; k < CPMinCost[0][0]; k++)
    {
        MCost = INF;

        for (i = 1; i <= CPMinCost[0][0]; i++)
        {
            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (ColoredArcs[i][j] == 1)
                {
                    if (CPMinCost[i][j] < MCost)
                    {
                        MCost = CPMinCost[i][j];
                        u = i;
                        v = j;
                    }
                }
            }
        }
        if (u < 0)
        {
            int a = 0;
            printf("Error in GetMinCostTree of FH");
            exit(0);
        }

        if (!Visited[u])
        {
            Visited[u] = 1;

            for (i = 1; i <= CPMinCost[0][0]; i++)
            {
                if (CPMinCost[u][i] < INF && !Visited[i])
                {
                    ColoredArcs[u][i] = 1;
                }

                if (CPMinCost[i][u] < INF && !Visited[i])
                {
                    ColoredArcs[i][u] = 1;
                }
            }
        }
        else
        {
            Visited[v] = 1;

            for (i = 1; i <= CPMinCost[0][0]; i++)
            {
                if (CPMinCost[v][i] < INF && !Visited[i])
                {
                    ColoredArcs[v][i] = 1;
                }

                if (CPMinCost[i][v] < INF && !Visited[i])
                {
                    ColoredArcs[i][v] = 1;
                }
            }
        }

        CPMCTree[u][v] = 1;
        CPMCTree[v][u] = 1;
        ColoredArcs[u][v] = 2;
        ColoredArcs[v][u] = 2;

        for (i = 1; i <= CPMinCost[0][0]; i++)
        {
            if (Visited[i])
                continue;

            MCost = INF;

            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (CPMinCost[i][j] < INF && Visited[j])
                {
                    ColoredArcs[i][j] = 0;

                    if (CPMinCost[i][j] < MCost)
                    {
                        MCost = CPMinCost[i][j];
                        u = i;
                        v = j;
                    }
                }
            }

            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (CPMinCost[j][i] < INF && Visited[j])
                {
                    ColoredArcs[j][i] = 0;

                    if (CPMinCost[j][i] < MCost)
                    {
                        MCost = CPMinCost[j][i];
                        u = j;
                        v = i;
                    }
                }
            }

            ColoredArcs[u][v] = 1;
            ColoredArcs[v][u] = 1;
        }
    }
}

void GetEulerRoute(int *EulerRoute, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes)
{
    int i, j;
    int backplace;

    EulerRoute[0] = 0;

    FindCircuit(EulerRoute, 1, AdMatrix, Nodes);

    for (backplace = 1; backplace <= EulerRoute[0]; backplace++)
    {
        if (EulerRoute[backplace] == 1)
            break;
    }

    backplace --;

    int TmpER[EulerRoute[0]+1];
    AssignArray(EulerRoute, TmpER);

    for (i = 1; i <= EulerRoute[0]-backplace; i++)
    {
        EulerRoute[i] = TmpER[i+backplace];
    }

    for (i = 1; i <= backplace; i++)
    {
        EulerRoute[EulerRoute[0]-backplace+i] = TmpER[i+1];
    }
}

void FindCircuit(int *EulerRoute, int CurrentNode, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes)
{
    int i, j, k;
    int Neighbors[Nodes[0]];
    Neighbors[0] = 0;

    for (i = 1; i <= Nodes[0]; i++)
    {
        if (AdMatrix[CurrentNode][Nodes[i]])
        {
            Neighbors[0] ++;
            Neighbors[Neighbors[0]] = Nodes[i];
        }
    }

    if (Neighbors[0] == 0)
    {
        EulerRoute[0] ++;
        EulerRoute[EulerRoute[0]] = CurrentNode;
    }
    else
    {
        while(Neighbors[0] > 0)
        {
            k = RandChoose(Neighbors[0]);

            AdMatrix[CurrentNode][Neighbors[k]] --;
            AdMatrix[Neighbors[k]][CurrentNode] --;

            FindCircuit(EulerRoute, Neighbors[k], AdMatrix, Nodes);

            Neighbors[0] = 0;
            for (i = 1; i <= Nodes[0]; i++)
            {
                if (AdMatrix[CurrentNode][Nodes[i]])
                {
                    Neighbors[0] ++;
                    Neighbors[Neighbors[0]] = Nodes[i];
                }
            }
        }

        EulerRoute[0] ++;
        EulerRoute[EulerRoute[0]] = CurrentNode;
    }
}

void EvenAllOddNodes(int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *OddNodes)
{
    int i, j, k;
    int LeftNodes[vertex_num+1];
    memset(LeftNodes, 0, sizeof(LeftNodes));
    AssignArray(OddNodes, LeftNodes);
//    for (i = 1; i <= OddNodes[0]; i++)
//    {
//        printf("%d \t", OddNodes[i]);
//    }
//    printf("1: %d, %d \n", OddNodes[0], LeftNodes[0]);

    struct Link
    {
        int LinkNode1Pos;
        int LinkNode2Pos;
        int LinkCost;
    };

    struct Link TmpLink, BestLink;

    for (i = 1; i <= OddNodes[0]/2; i++)
    {
        BestLink.LinkCost = INF;
        for (j = 1; j < LeftNodes[0]; j++)
        {
            for (k = j+1; k <= LeftNodes[0]; k++)
            {
                TmpLink.LinkNode1Pos = j;
                TmpLink.LinkNode2Pos = k;
                TmpLink.LinkCost = min_cost[LeftNodes[j]][LeftNodes[k]];
//                printf("%d %d %d\n", LeftNodes[j], LeftNodes[k], TmpLink.LinkCost);

                if (TmpLink.LinkCost < BestLink.LinkCost)
                    BestLink = TmpLink;
            }
        }

        //printf("Link %d and %d\n", LeftNodes[BestLink.LinkNode1Pos], LeftNodes[BestLink.LinkNode2Pos]);

        AdMatrix[LeftNodes[BestLink.LinkNode1Pos]][LeftNodes[BestLink.LinkNode2Pos]] ++;
        AdMatrix[LeftNodes[BestLink.LinkNode2Pos]][LeftNodes[BestLink.LinkNode1Pos]] ++;

//        printf("2: %d, %d, %d, %d \n", OddNodes[0], LeftNodes[0], BestLink.LinkNode1Pos, BestLink.LinkNode2Pos);
        if (BestLink.LinkNode1Pos < BestLink.LinkNode2Pos)
        {
            delete_element(LeftNodes, BestLink.LinkNode2Pos);
            delete_element(LeftNodes, BestLink.LinkNode1Pos);
        }
        else
        {
            delete_element(LeftNodes, BestLink.LinkNode1Pos);
            delete_element(LeftNodes, BestLink.LinkNode2Pos);
        }
    }
//    printf("End...\n");
}