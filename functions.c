//
// Created by hao on 01/06/2020.
//

#include <stdio.h>
#include "functions.h"


void mod_dijkstra()
{
    memset(min_cost, 0, sizeof(min_cost));
    memset(shortest_path, 0, sizeof(shortest_path));
    int i, j, k, m, minimum;
//    printf("Dijastra\n");
    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            shortest_path[i][j][0] = 1;
            shortest_path[i][j][1] = i;
            min_cost[i][j] = INF;
        }
    }

    int mark[MAX_NODE_TAG_LENGTH], dist[MAX_NODE_TAG_LENGTH], dist1[MAX_NODE_TAG_LENGTH], nearest_neighbor[MAX_NODE_TAG_LENGTH];

    for (i = 1; i <= vertex_num; i++)
    {
        mark[i] = 1;

        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            mark[j] = 0;
            dist[j] = trav_cost[i][j];
            dist1[j] = dist[j];
        }

        for (k = 1; k < vertex_num; k++)
        {
            minimum = INF;
            nearest_neighbor[0] = 0;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == INF)
                    continue;

                if (dist1[j] < minimum)
                    minimum = dist1[j];
            }

            if (minimum == INF)
                continue;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == minimum)
                {
                    nearest_neighbor[0] ++;
                    nearest_neighbor[nearest_neighbor[0]] = j;
                }
            }

            int v = nearest_neighbor[1];
            dist1[v] = INF;
            mark[v] = 1;

            if (shortest_path[i][v][0] == 0 || (shortest_path[i][v][0] > 0 && shortest_path[i][v][shortest_path[i][v][0]] != v))
            {
                shortest_path[i][v][0] ++;
                shortest_path[i][v][shortest_path[i][v][0]] = v;
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (minimum+trav_cost[v][j] < dist[j])
                {
                    dist[j] = minimum+trav_cost[v][j];
                    dist1[j] = minimum+trav_cost[v][j];
                    for (m = 0; m <= shortest_path[i][v][0]; m++)
                    {
                        shortest_path[i][j][m] = shortest_path[i][v][m];
                    }
                }
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (j == i)
                    continue;

                min_cost[i][j] = dist[j];
            }
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (shortest_path[i][j][0] == 1)
                shortest_path[i][j][0] = 0;
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        shortest_path[i][i][0] = 1;
        shortest_path[i][i][1] = i;
        min_cost[i][i] = 0;
    }
}


void update_cost(const Task *inst_tasks, const Arc *inst_arcs)
{

    memset(trav_cost, 0, sizeof(trav_cost));
    // update trav_cost, serve_cost, min_cost after each dynamic change
    for (int i=1; i<=vertex_num; i++)
    {
        for (int j=1; j<=vertex_num; j++)
        {
            trav_cost[i][j] = INF;
            serve_cost[i][j] = 0;
        }
    }

    trav_cost[1][0] = 0;
    trav_cost[0][1] = 0;

    for (int i=1; i<=task_num; i++)
    {
        serve_cost[inst_tasks[i].head_node][inst_tasks[i].tail_node] = inst_tasks[i].serv_cost;
    }

    for (int i=1; i<=total_arc_num; i++)
    {
        // printf("%d, %d: %d\n", inst_arcs[i].head_node, inst_arcs[i].tail_node, inst_arcs[i].trav_cost);
        trav_cost[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }
//    printf("\n");
}



void construct_virtual_task(const Task *inst_tasks, Task *tasks_vt, const int *stop, const int *remain_capacity)
{

    int i;

    int actual_req_edge_num = req_edge_num;
    req_edge_num += stop[0];
    task_num = 2 * req_edge_num;

    
    for (i = 1; i <= actual_req_edge_num; i++)
    {
        tasks_vt[i].head_node = inst_tasks[i].head_node;
        tasks_vt[i].tail_node = inst_tasks[i].tail_node;
        tasks_vt[i].dead_cost = inst_tasks[i].dead_cost;
        tasks_vt[i].serv_cost = inst_tasks[i].serv_cost;
        tasks_vt[i].demand = inst_tasks[i].demand;
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 0;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 0;
    }
    if (stop[0] == 0)
    {
        return;
    }
    for (i = actual_req_edge_num+1; i <= req_edge_num; i++)
    {
        tasks_vt[i].head_node = DEPOT;
        tasks_vt[i].tail_node = stop[i-actual_req_edge_num];
        tasks_vt[i].dead_cost = 0;
        tasks_vt[i].serv_cost = min_cost[tasks_vt[i].head_node][tasks_vt[i].tail_node];
        tasks_vt[i].demand = capacity - remain_capacity[i-actual_req_edge_num];
        if (tasks_vt[i].demand == 0)
        {
            tasks_vt[i].demand = 1;
        }
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 1;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 1;
    }
    tasks_vt[0].head_node = DEPOT;
    tasks_vt[0].tail_node = DEPOT;
    tasks_vt[0].dead_cost = 0;
    tasks_vt[0].serv_cost = 0;
    tasks_vt[0].demand = 0;
    tasks_vt[0].inverse = 0;


}



void inherit_solution(const int *prev_seq, int *new_seq, const Task *prev_inst_tasks, const Task *new_inst_tasks)
{
    int i, j;
    int AdMatrix[300][300];
    memset(AdMatrix, 0, sizeof(AdMatrix));
    for (i = 1; i <= req_edge_num; i++ )
    {
        AdMatrix[new_inst_tasks[i].head_node][new_inst_tasks[i].tail_node] = i;
        AdMatrix[new_inst_tasks[i].tail_node][new_inst_tasks[i].head_node] = i+req_edge_num;
    }
    for (i = 1; i <= prev_seq[0]; i++)
    {
        printf("%d ", prev_seq[i]);
        printf("%d(%d, %d)  ", prev_seq[i], prev_inst_tasks[prev_seq[i]].head_node, prev_inst_tasks[prev_seq[i]].tail_node);
    }
    printf("\n");
//    int new_seq[MAX_TASKS_TAG_LENGTH];
//    memset(new_seq, 0, sizeof(new_seq));
    for (i = 1; i <= prev_seq[0]; i++)
    {
        if(prev_seq[i] == 0)
        {
            new_seq[0]++;
            new_seq[new_seq[0]] = prev_seq[i];
        } else {
            if (AdMatrix[prev_inst_tasks[prev_seq[i]].head_node][prev_inst_tasks[prev_seq[i]].tail_node])
            {
                new_seq[0]++;
                new_seq[new_seq[0]] = AdMatrix[prev_inst_tasks[prev_seq[i]].head_node][prev_inst_tasks[prev_seq[i]].tail_node];
            }
        }
    }
//    return new_seq;
    for (i = 1; i <= new_seq[0]; i++)
    {
        printf("%d ", new_seq[i]);
    }
    printf("\n");
}


int repair_solution_greedy_insertion(Individual *solution, int *remain_seq, const int *stop, const Task *inst_tasks_vt)
{
    int i, j;
    int remain_seq_num = 0;

    int used[MAX_TASKS_TAG_LENGTH];
    memset(used, 0, sizeof(used));

    int new_seq[MAX_TASK_SEQ_LENGTH];
    memset(new_seq, 0, sizeof(new_seq));

    

    // check stop points with routes
    int vtIdx = req_edge_num - stop[0];
    int actual_req_edge_num = vtIdx;
    for (i = 1; i < remain_seq[0]; i++)
    {
        if (remain_seq[i] < 0)
            continue;
        
        if (remain_seq[i] > actual_req_edge_num)
        {
            remain_seq[i] += stop[0];
        }
        if (remain_seq[i] == 0)
        {
            vtIdx ++;
            remain_seq[i] = vtIdx;
        }

        new_seq[0] ++;
        new_seq[new_seq[0]] = remain_seq[i];
        used[remain_seq[i]] = 1;
        used[inst_tasks_vt[remain_seq[i]].inverse] = 1;
        used[0]++;

    }
    remain_seq_num = new_seq[0] - stop[0];

    int increase_cost, minimal_cost;
    int minimal_index, flag;
    for(i = 1; i <= req_edge_num; i++)
    {
        if (used[i])
            continue;

        increase_cost = min_cost[DEPOT][inst_tasks_vt[i].head_node]
            + min_cost[inst_tasks_vt[i].tail_node][inst_tasks_vt[new_seq[1]].head_node];
        minimal_cost = increase_cost;
        minimal_index = 1;

        

        for (j = 1; j <= new_seq[0]; j++)
        {
            increase_cost = min_cost[inst_tasks_vt[new_seq[j]].tail_node][inst_tasks_vt[i].head_node]
            + min_cost[inst_tasks_vt[i].tail_node][inst_tasks_vt[new_seq[j+1]].head_node];

            if (increase_cost < minimal_cost)
            {
                minimal_cost = increase_cost;
                minimal_index = j+1;
            }
        }

        flag = 0;
        increase_cost = min_cost[DEPOT][inst_tasks_vt[i].tail_node]
            + min_cost[inst_tasks_vt[i].head_node][inst_tasks_vt[new_seq[1]].head_node];
        if (increase_cost < minimal_cost)
        {
            minimal_cost = increase_cost;
            minimal_index = 1;
            flag = 1;
        }

        for (j = 1; j <= new_seq[0]; j++)
        {
            increase_cost = min_cost[inst_tasks_vt[new_seq[j]].tail_node][inst_tasks_vt[i].tail_node]
                            + min_cost[inst_tasks_vt[i].head_node][inst_tasks_vt[new_seq[j+1]].head_node];

            if (increase_cost < minimal_cost)
            {
                minimal_cost = increase_cost;
                minimal_index = j+1;
                flag = 1;
            }
        }
        if (flag)
        {
            add_element(new_seq, inst_tasks_vt[i].inverse, minimal_index);
//            printf("%d %d\n", inst_tasks_vt[i].inverse, new_seq[0]);
        } else {
            add_element(new_seq, i, minimal_index);
        }
        used[i] = 1;
        used[inst_tasks_vt[i].inverse] = 1;
        used[0]++;
    }

    memset(solution->Sequence, 0, sizeof(solution->Sequence));
    memset(solution->Assignment, 0, sizeof(solution->Assignment));
    for (i = 1; i <= new_seq[0]; i++)
    {
        if (new_seq[i] == 0)
        {
            continue;
        }
        solution->Assignment[0]++;
        solution->Assignment[solution->Assignment[0]] = new_seq[i];
    }
    return remain_seq_num;
}



void repair_solution_with_new_vehicles(int *new_seq, Individual *solution, const int *stop, Vehicles *info, const Task *inst_tasks_vt)
{
    int i, j;


    int used[MAX_TASKS_TAG_LENGTH];
    memset(used, 0, sizeof(used));

    // check stop points with routes
    int vtIdx = req_edge_num - stop[0];
    int actual_req_edge_num = vtIdx;

    solution->Sequence[0] = 0;
//    solution->Sequence[1] = 0;
    for (i = 1; i < new_seq[0]; i++)
    {
        if (new_seq[i] < 0)
            continue;

        if (new_seq[i] > actual_req_edge_num)
        {
            new_seq[i] += stop[0];
        }
        if (new_seq[i] == 0)
        {
            solution->Sequence[0]++;
            solution->Sequence[solution->Sequence[0]] = 0;
            vtIdx ++;
            new_seq[i] = vtIdx;
        }
        solution->Sequence[0]++;
        solution->Sequence[solution->Sequence[0]] = new_seq[i];
        used[new_seq[i]] = 1;
        used[inst_tasks_vt[new_seq[i]].inverse] = 1;
        used[0]++;
        if (solution->Sequence[solution->Sequence[0]] == -1)
        {
            int a = 0;
        }
    }


    int remain_req_edegs[MAX_TASKS_TAG_LENGTH];
    memset(remain_req_edegs, 0, sizeof(remain_req_edegs));
    for (i = 1; i <= req_edge_num; i++)
    {
        if (used[i])
            continue;

        remain_req_edegs[0] ++;
        remain_req_edegs[remain_req_edegs[0]] = i;
//        printf("%d ", i);
    }
//    printf("\n");

    Task sub_graph[MAX_TASKS_TAG_LENGTH];
    for (i = 1; i <= remain_req_edegs[0]; i++)
    {
        sub_graph[i].head_node = inst_tasks_vt[remain_req_edegs[i]].head_node;
        sub_graph[i].tail_node = inst_tasks_vt[remain_req_edegs[i]].tail_node;
        sub_graph[i].demand = inst_tasks_vt[remain_req_edegs[i]].demand;
        sub_graph[i].serv_cost = inst_tasks_vt[remain_req_edegs[i]].serv_cost;
        sub_graph[i].dead_cost = inst_tasks_vt[remain_req_edegs[i]].dead_cost;
        sub_graph[i].inverse = i + remain_req_edegs[0];

        sub_graph[i+remain_req_edegs[0]].head_node = sub_graph[i].tail_node;
        sub_graph[i+remain_req_edegs[0]].tail_node = sub_graph[i].head_node;
        sub_graph[i+remain_req_edegs[0]].demand = sub_graph[i].demand;
        sub_graph[i+remain_req_edegs[0]].dead_cost = sub_graph[i].dead_cost;
        sub_graph[i+remain_req_edegs[0]].serv_cost = sub_graph[i].serv_cost;
        sub_graph[i+remain_req_edegs[0]].inverse = i;
    }

    // change req_edge_num, task_num, vehicle_num, temporarily
    req_edge_num = remain_req_edegs[0];
    task_num = 2*req_edge_num;
    vehicle_num = vehicle_num - stop[0];

    Individual remain_solution;
    MAENS(sub_graph, &remain_solution);
    for (i = 1; i <= remain_solution.Sequence[0]; i++)
    {

        if (remain_solution.Sequence[i] == 0)
        {
            solution->Sequence[0]++;
            solution->Sequence[solution->Sequence[0]] = 0;
        } else if (remain_solution.Sequence[i] <= remain_req_edegs[0]) {
            solution->Sequence[0]++;
            solution->Sequence[solution->Sequence[0]] = remain_req_edegs[remain_solution.Sequence[i]];
        } else if (remain_solution.Sequence[i] > remain_req_edegs[0]) {
            solution->Sequence[0]++;
            solution->Sequence[solution->Sequence[0]] = inst_tasks_vt[remain_req_edegs[sub_graph[remain_solution.Sequence[i]].inverse]].inverse;
        }

//        if (remain_solution.Sequence[i] != 0)
//        {
//            printf("%d ", remain_solution.Sequence[i]);
//        }
    }
    // printf("1234flag\n");

    // check the total cost
    solution->TotalCost = get_task_seq_total_cost(solution->Sequence, inst_tasks_vt);

    // recover req_edge_num, task_num, vehicle_num
    vehicle_num += stop[0];
    req_edge_num = actual_req_edge_num + stop[0];
    task_num = 2 * req_edge_num;
//    check_tasks(solution->Sequence, req_edge_num, inst_tasks_vt);
//    printf("aaaaa");

}

int back_and_new(const int *stop, const Task *inst_tasks, const int seed)
{
    req_edge_num = req_edge_num - stop[0];
    task_num = req_edge_num * 2;
    Individual solution;
    srand(seed);
    LMA(inst_tasks, &solution);
    check_solution_valid(solution, inst_tasks);
    req_edge_num = req_edge_num + stop[0];
    task_num = req_edge_num * 2;
    
    int back_cost = 0;
    for (int i=1; i <= stop[0]; i++)
    {
        back_cost += min_cost[stop[i]][DEPOT];
    }

    return back_cost + solution.TotalCost;
}

void check_tasks(const int *seq, int req_num, const Task *inst_tasks)
{
    int i, j, t;
    int tem[seq[0]+1];
    memset(tem, 0, sizeof(tem));
    for (i = 1; i <= seq[0]; i++)
    {
        if (seq[i] == 0)
            continue;

        tem[0] ++;
        if (seq[i] > req_num)
        {
            tem[tem[0]] = inst_tasks[seq[i]].inverse;
        } else {
            tem[tem[0]] = seq[i];
        }
    }
    for (i = 1; i < tem[0]; i++)
    {
        for (j = i; j <= tem[0]; j++)
        {
            if (tem[i] > tem[j])
            {
                t = tem[i];
                tem[i] = tem[j];
                tem[j] = t;
            }
        }
    }

    for (i = 1; i <= tem[0]; i++)
    {
        printf("%d ", tem[i]);
    }
    printf("\n");
}

int check_load_num(int *seq)
{
    int load_num = 0;
    for (int i=1; i < seq[0]; i++)
    {
        if(seq[i] == 0)
        {
            load_num ++;
        }
    }
    // printf("load_num: %d \n", load_num);
    return load_num;
}

void showSeq(int *seq)
{
    for (int i=0; i <= seq[0]; i++)
    {
        printf("%d\t", seq[i]);
    }
    printf("\n");
}

int check_task_valid(int *seq)
{
    int i;
    for (i = 1; i <= seq[0]; i++)
    {
        if(seq[i] > 2*req_edge_num)
        {
            printf("tasks valid, req_edge_num: %d, seq: %d\n", req_edge_num, seq[i]);
            return 0;
        }
    }
    return 1;
}

void clear_solution(Individual *solution)
{
    memset(solution->Sequence, 0, sizeof(solution->Sequence));
    memset(solution->Assignment, 0, sizeof(solution->Assignment));
    memset(solution->Loads, 0, sizeof(solution->Loads));
    solution->TotalCost = 0;
    solution->TotalVioLoad = 0;
    solution->Fitness = 0;
}

int get_totoal_loads(int *seq, const Task *inst_tasks)
{
    int load = 0;
    for (int i = 1; i <= seq[0]; i++)
    {
        printf("%d\t", inst_tasks[seq[i]].demand);
        load += inst_tasks[seq[i]].demand;
    }
    printf("\n");
    return load;
}

void get_each_load(int *seq, const Task *inst_tasks)
{
    int load = 0, total_vio_load = 0;
    for (int i = 1; i <= seq[0]; i++)
    {
        if(seq[i] == 0)
        {
            printf("%d\t", load);
            if (load > capacity)
            {
                total_vio_load += load - capacity;
            }
            load = 0;
            continue;
        }
        load += inst_tasks[seq[i]].demand;
    }
    if(total_vio_load > 0)
    {
        printf("violation.\n");
        // exit(0);
    }
    printf("\n total_vio_load: %d \n", total_vio_load);
}

void check_solution_valid(Individual solution, const Task *inst_task)
{
    int i, j;
    int used[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    for(i = 1; i <= solution.Sequence[0]; i++)
    {
        if (solution.Sequence[i] == 0)
        {
            continue;
        }
        if (solution.Sequence[i] <= req_edge_num)
        {
            used[solution.Sequence[i]] = 1;
        } else
        {
            used[inst_task[solution.Sequence[i]].inverse] = 1;
        }
    }
    int flag = 0;
    // printf("lack of: ");
    for (i = 1; i <= req_edge_num; i++)
    {
        if(used[i] == 0)
        {
            printf("%d \t", i);
            flag = 1;
        }
    }
    // printf("\n");
    // get_each_load(solution.Sequence, inst_task);
    if ( solution.TotalCost != get_task_seq_total_cost(solution.Sequence, inst_task))
    {
        flag = 1;
        printf("solution's cost error\n");
    }
    if (flag)
    {
        exit(0);
    }
}

void check_seq_valid(Individual solution, const Task *inst_task)
{
    int i, j;
    int used[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    for(i = 1; i <= solution.Sequence[0]; i++)
    {
        if (solution.Sequence[i] == 0)
        {
            continue;
        }
        if (solution.Sequence[i] <= req_edge_num)
        {
            used[solution.Sequence[i]] = 1;
        } else
        {
            used[inst_task[solution.Sequence[i]].inverse] = 1;
        }
    }
    int flag = 0;
    // printf("lack of: ");
    for (i = 1; i <= req_edge_num; i++)
    {
        if(used[i] == 0)
        {
            printf("lack: %d \t", i);
            flag = 1;
        }
    }
    if (flag)
    {
        exit(0);
    }
    // printf("\n");
}



