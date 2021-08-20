//
// Created by hao on 08/06/2020.
//

#include "simulator.h"

int choose_stop_time(int *cost);
void process_solution(Individual *Solution, const Task *inst_tasks_vt);

void nextScenario(Individual *Solution, Task *inst_tasks_vt, Task *inst_tasks, Arc *inst_arcs, Vehicles *state, unsigned int seed)
{
    int tau = 400;
    int serve_tasks[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];;
    memset(serve_tasks, 0, sizeof(serve_tasks));

    srand(seed);
    process_solution(Solution, inst_tasks_vt);
    // printf("seed: %d \n", seed);
    executeSolution(Solution, tau, state, serve_tasks, inst_tasks_vt); // inst_tasks_vt: old graph
    dynamicChange(inst_tasks, inst_arcs, serve_tasks, state, state->remain_seqs, seed); // inst_tasks: new graph


}

void process_solution(Individual *Solution, const Task *inst_tasks_vt)
{
    int i, j;
    for (i =1; i < Solution->Sequence[0]; i++)
    {
        if (inst_tasks_vt[Solution->Sequence[i]].vt > 0 && Solution->Sequence[i-1] != 0)
        {
            add_element(Solution->Sequence, 0, i);
        }
    }
}



int choose_stop_time(int *cost1)
{
    int i, j, tmp;
    int cost[cost1[0]+1];
    memcpy(cost, cost1, sizeof(int)*(cost1[0]+1));
    for (i = 1; i < cost[0]; i++)
    {
        for(j = i+1; j <= cost[0]; j++)
        {
            if (cost[i] < cost[j])
            {
                tmp = cost[j];
                cost[j] = cost[i];
                cost[i] = tmp;
            }
        }
    }
    int idx;
    if (cost[0] < 10)
    {
        // when route_num < 10, control the outside vehicles as [n/2-1, n/2, n/2+1]
        idx = rand_choose(3) - 2 + cost[0] / 2;
        if (idx < 2)
        {
            idx ++;
        }
    } else
    {
        // when route_num > 10, control the outside vehicles as [2, 6].
        idx = rand_choose(5) + 1;
    }
    switch (cost[0])
    {
        case 0:
            idx = 0;
            break;
        case 1:
            idx = 1;
            break;
        case 2: 
            idx = 1;
            break;
        case 3:
            idx = 2;
            break;
        case 4:
            idx = 2;
            break;
        default:
            break;
    }

    int r = rand();
    int tau = (int)(( 0.2 * r / RAND_MAX ) * (cost[idx+1] - cost[idx])) + cost[idx];
    return tau;
}

/*
 input: start point, solution,
 output: stop point, remain capacity;
 */
void executeSolution(Individual *Solution, int tau1, Vehicles *state, int (*serve_tasks)[MAX_NODE_TAG_LENGTH], const Task * inst_tasks_vt)
{
    //    int tau = 100;

    int i, j, k;

    for (i = 1; i <= Solution->Loads[0]; i++)
    {
        if (Solution->Loads[i] > capacity)
        {
            printf("Exceed the capacity %d\n", Solution->Loads[i]);
        }
    }

    int Route[101][MAX_TASK_SEQ_LENGTH];
    int new_route[101];
    int out_route[101];
    memset(Route, 0, sizeof(Route));
    memset(new_route, 0, sizeof(new_route));
    memset(out_route, 0, sizeof(out_route));

    int Positions[101];
    find_ele_positions(Positions, Solution->Sequence, 0);
    Route[0][0] = Positions[0]-1;
    for (i=1; i < Positions[0]; i++)
    {
        AssignSubArray(Solution->Sequence, Positions[i], Positions[i+1], Route[i]);
        if (inst_tasks_vt[Solution->Sequence[Positions[i]+1]].vt > 0)
        {
            out_route[0] ++;
            out_route[out_route[0]] = i;
        } else {
            new_route[0] ++;
            new_route[new_route[0]] = i;
        }
    }
    if (out_route[0] > vehicle_num)
    {
        printf("outside vehicle is greater than maximal vehicles \n");
        exit(0);
    }

    // the cost of out routes should remove the virtual part.
   int cost[101];
   memset(cost, 0, sizeof(cost));
   cost[0] = 0;
   for (i = 1; i <= Route[0][0]; i++)
   {
       cost[0] ++;
       if (inst_tasks_vt[Route[i][2]].vt > 0)
       {
           cost[cost[0]] = get_task_seq_total_cost(Route[i], inst_tasks_vt) - inst_tasks_vt[Route[i][2]].serv_cost;
       } else {
           cost[cost[0]] = get_task_seq_total_cost(Route[i], inst_tasks_vt);
       }
   }

   int tmp;
   for (i = 1; i < out_route[0]; i++)
   {
       for (j = i+1; j <= out_route[0]; j++)
       {
           if (cost[out_route[i]] > cost[out_route[j]])
           {
               tmp = out_route[i];
               out_route[i] = out_route[j];
               out_route[j] = tmp;
           }
       }
   }

    for (i = 1; i < new_route[0]; i++)
    {
        for (j = i+1; j <= new_route[0]; j++)
        {
            if (cost[new_route[i]] < cost[new_route[j]])
            {
                tmp = new_route[i];
                new_route[i] = new_route[j];
                new_route[j] = tmp;
            }
        }
    }

    int schedule[101][MAX_TASK_SEQ_LENGTH];
    memset(schedule, 0, sizeof(schedule));
    schedule[0][0] = vehicle_num;

    if (new_route[0] + out_route[0] < vehicle_num)
    {
        schedule[0][0] = new_route[0] + out_route[0];
    }

    for(i = 1; i <= out_route[0]; i++)
    {
        JoinArray(schedule[i], Route[out_route[i]]);
        schedule[i][0] --;
    }

    k = 0;
    i --;
    while (1)
    {
        i ++;
        k ++;
        if (k > new_route[0])
            break;

        JoinArray(schedule[i], Route[new_route[k]]);
        schedule[i][0] --;
        if (i == vehicle_num)
        {
            i = 0;
        }
    }

    // the schedule has been determined

    // the schedule start to be served


    int tour[101][5000];
    memset(tour, 0, sizeof(tour));
    tour[0][0] = schedule[0][0];
    int schedule_cost[101];
    memset(schedule_cost, 0, sizeof(schedule_cost));
    for (i = 1; i <= schedule[0][0]; i++)
    {
        schedule[i][0] ++;
        schedule_cost[i] = get_task_seq_total_cost(schedule[i], inst_tasks_vt);
    }
    schedule_cost[0] = schedule[0][0];

    // determine tau from schedule_cost
    // make half vehicles back to the depot
    for (i = 1; i <= schedule[0][0]; i++)
    {
        if (inst_tasks_vt[schedule[i][2]].vt > 0)
        {
            schedule_cost[i] -= inst_tasks_vt[schedule[i][2]].serv_cost;
        }
    }
    // int tau = choose_stop_time(schedule_cost);

    int task_cost[101][100];
    memset(task_cost, 0, sizeof(task_cost));
    task_cost[0][0] = schedule[0][0];
    for (i = 1; i <= schedule[0][0]; i++)
    {
        task_cost[i][0] = schedule[i][0];
        for(j = 2; j <= schedule[i][0]; j++)
        {
            task_cost[i][j] = task_cost[i][j-1] + min_cost[inst_tasks_vt[schedule[i][j-1]].tail_node][inst_tasks_vt[schedule[i][j]].head_node] + inst_tasks_vt[schedule[i][j]].serv_cost;
        }
    }

    int remain_load[101][100];
    memset(remain_load, 0, sizeof(remain_load));
    remain_load[0][0] = schedule[0][0];
    for (i = 1; i <= schedule[0][0]; i++)
    {
        remain_load[i][0] = schedule[i][0];
        remain_load[i][1] = capacity;
        for(j = 2; j <= schedule[i][0]; j++)
        {
            if (schedule[i][j] == 0)
            {
                remain_load[i][j] = capacity;
            } else {
                remain_load[i][j] = remain_load[i][j-1] - inst_tasks_vt[schedule[i][j]].demand;
            }
        }
    }
    // state->remain_seqs[0] = 1;
    // state->remain_seqs[1] = 0;

    // the code below is to randomly select a stop point in a route
    int tmp_tour[500];
    int stop, stop_task_index, start, end;
    double threshold_lb = capacity * remain_cap_ratio_lb;
    double threshold_ub = capacity * remain_cap_ratio_ub;

    int stop_num;
    if (schedule [0][0] > 6)
    {
        stop_num = rand_choose(6);
    } else
    {
        stop_num = rand_choose(schedule[0][0]);
    }
    // stop_num = 1;
    printf("stop num: %d total: %d\n", stop_num, schedule[0][0]);
    
     
    int select_vehicles[101];
    memset(select_vehicles, 0, sizeof(select_vehicles));
    rand_perm(select_vehicles, schedule[0][0]);

    int select[101];
    memset(select, 0, sizeof(select));
    for (i = 1; i <= stop_num; i++)
    {
        select[select_vehicles[i]] = 1;
    }
    

    for (i = 1; i <= schedule[0][0]; i++)
    {
        if ( select[i] )
        {

            int stop_avail[100];
            memset(stop_avail, 0, sizeof(stop_avail));
            if (schedule[i][0] > 100)
            {
                printf("memory error in simulator.c line 288 \n");
                exit(0);
            }
            int kk;
            for (kk = 1; kk <= remain_load[i][0]; kk++)
            {
                if (remain_load[i][kk] > threshold_lb && remain_load[i][kk] < threshold_ub)
                {
                    stop_avail[0] ++;
                    stop_avail[kk] = stop_avail[0];
                }
            }
            if (stop_avail[0] == 0){
                stop_task_index = (int)(remain_load[i][0]*0.15);
            } else
            {
                int stop_index = rand_choose(stop_avail[0]);
                for (kk = 1; kk <= remain_load[i][0]; kk++)
                {
                    if (stop_avail[kk] == stop_index)
                    {
                        stop_task_index = kk;
                        break;
                    }
                }
            }
            if (stop_task_index == 0)
            {
                stop_task_index ++;
            }


            // stop_task_index = lowerindex + rand_choose((int)(upperindex - lowerindex));
            // stop_task_index = rand_choose((int)(remain_load[i][0]*0.7));
            start = inst_tasks_vt[schedule[i][stop_task_index]].tail_node;
            end = inst_tasks_vt[schedule[i][stop_task_index+1]].head_node;
            AssignArray(shortest_path[start][end], tmp_tour);
            stop = tmp_tour[rand_choose(tmp_tour[0])];
            if (stop == 1)
            {
                stop = tmp_tour[tmp_tour[0] - 1];
            }
            state->stop[0] ++;
            state->stop[state->stop[0]] = stop;
            state->remain_capacity[0] ++;
            state->remain_capacity[state->remain_capacity[0]] = remain_load[i][stop_task_index];
            // printf("%d \n", state->remain_capacity[state->remain_capacity[0]]);
            state->remain_seqs[0] ++;
            state->remain_seqs[state->remain_seqs[0]] = 0;
            for (j = stop_task_index; j <= schedule[i][0]; j++)
            {
                if (schedule[i][j] != 0)
                {
                    state->remain_seqs[0] ++;
                    state->remain_seqs[state->remain_seqs[0]] = schedule[i][j];
                }
            }
        }
    }
    state->remain_seqs[0] ++;
    state->remain_seqs[state->remain_seqs[0]] = 0;



    /* 
    // the code below is execute solution according to the cost(distance) of each route.
    int tmp_tour[500];

    int currTask, currNode, currLoad, start, start_index, dis, done;
    for (i = 1; i <= schedule[0][0]; i++)
    {
        if (inst_tasks_vt[schedule[i][2]].vt > 0)
        {
            start = inst_tasks_vt[schedule[i][2]].tail_node;
            start_index = 3;
            currLoad = inst_tasks_vt[schedule[i][2]].demand;
        } else {
            start = DEPOT;
            start_index = 2;
            currLoad = 0;
        }
        dis = 0;
        done = 0;
        for (j = start_index; j <= schedule[i][0]; j++) // check the last task of schedule[i], it should be 0
        {
            // printf("%d\t", dis);
            currTask = schedule[i][j];
            currNode = inst_tasks_vt[currTask].head_node;
            dis += min_cost[start][currNode];
            if (dis > tau)
            {
                dis -= min_cost[start][currNode];
                AssignArray(shortest_path[start][currNode], tmp_tour);
                for (k = 1; k <= tmp_tour[0]; k++)
                {
                    currNode = tmp_tour[k];
                    dis += min_cost[start][currNode];
                    if (dis > tau)
                    {
                        done = 1;
                        break;
                    }
                    start = currNode;
                }
                break;
            }

            if (currTask == 0)
            {
                currLoad = 0;
                start = DEPOT;
                continue;
            }

            start = currNode;
            currNode = inst_tasks_vt[currTask].tail_node;
            dis += inst_tasks_vt[currTask].dead_cost;
            if (dis > tau)
            {
                done = 1;
                break;
            }
            currLoad += inst_tasks_vt[currTask].demand;
            if (inst_tasks_vt[currTask].vt == 0)
            {
                serve_tasks[inst_tasks_vt[currTask].head_node][inst_tasks_vt[currTask].tail_node] = 1;
                serve_tasks[inst_tasks_vt[currTask].tail_node][inst_tasks_vt[currTask].head_node] = 1;
            }
            start = currNode;
//            printf("%d ", currTask);
        }
        // printf("\n");
        if (done)
        {
            state->remain_seqs[0] ++;
            state->remain_seqs[state->remain_seqs[0]] = 0;
            for (; j <= schedule[i][0]; j++)
            {
                if (schedule[i][j] != 0)
                {
                    state->remain_seqs[0] ++;
                    state->remain_seqs[state->remain_seqs[0]] = schedule[i][j];
                }
            }
            state->stop[0] ++;
            state->stop[state->stop[0]] = start;
            state->remain_capacity[0] ++;
            state->remain_capacity[state->remain_capacity[0]] = capacity - currLoad;
//            printf("%d, %d, %d\n", stop[stop[0]], currLoad, remain_capacity[remain_capacity[0]]);
        }
    }
    state->remain_seqs[0] ++;
    state->remain_seqs[state->remain_seqs[0]] = 0;
    */
}
/*
 input: inst_tasks, inst_arcs
 output: new_inst_tasks, new_inst_tasks, new_req_edge_num, new_nonreq_edge_num, new_vertex_num;
 */
void dynamicChange(Task *inst_tasks, Arc *inst_arcs, const int (*serve_tasks)[MAX_NODE_TAG_LENGTH], Vehicles *info, int *unserved_seq, unsigned int seed)
{



    double p2, p3, p4, p5, p_bdrr, p_crr, p_crbb;
    p2 = 0.5; p3 = 0.9;  // related to cost
    p4 = 0.35; p5 = 0.35; // related to demand
    p_bdrr = 0.5; // breakdown road recover
    p_crbb = 0.6; // congestion road become better
    p_crr = 0.3; // congestion road recover

    int i, j;
    Edge graph[MAX_TASKS_TAG_LENGTH];
    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(AdMatrix, 0, sizeof(AdMatrix));


    int unServMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i = 1; i <= unserved_seq[0]; i++)
    {
        if (unserved_seq[i] != 0)
        {
            unServMatrix[inst_tasks[unserved_seq[i]].head_node][inst_tasks[unserved_seq[i]].tail_node] = i;
            unServMatrix[inst_tasks[unserved_seq[i]].tail_node][inst_tasks[unserved_seq[i]].head_node] = i + 10000;
        }
    }


    req_edge_num = edge_index[0];
    nonreq_edge_num = edge_index[1];


    for (i = 1; i <= req_edge_num; i++)
    {
        graph[i].head_node = inst_tasks[i].head_node;
        graph[i].tail_node = inst_tasks[i].tail_node;
        graph[i].trav_cost = inst_tasks[i].dead_cost;
        if (serve_tasks[graph[i].head_node][graph[i].tail_node] || serve_tasks[graph[i].tail_node][graph[i].head_node])
        {
            graph[i].demand = 0;
        } else{
            graph[i].demand = inst_tasks[i].demand;
        }

        if (unServMatrix[graph[i].head_node][graph[i].tail_node])
        {
            graph[i].unserved = unServMatrix[graph[i].head_node][graph[i].tail_node];
        } else {
            graph[i].unserved = 0;
        }

        graph[i].change = inst_arcs[i].change;
        graph[i].link = inst_arcs[i].link;
        if (graph[i].link > 2*req_edge_num)
        {
            graph[i].link -= req_edge_num;
        }
        AdMatrix[graph[i].head_node][graph[i].tail_node] = i;
        AdMatrix[graph[i].tail_node][graph[i].head_node] = i;
    }


    for (j=req_edge_num+1; j<=req_edge_num+nonreq_edge_num; j++)
    {
        i = j + req_edge_num;
        graph[j].head_node = inst_arcs[i].head_node;
        graph[j].tail_node = inst_arcs[i].tail_node;
        graph[j].trav_cost = inst_arcs[i].trav_cost;
        graph[j].demand = 0;
        graph[j].change = inst_arcs[i].change;
        graph[j].link = inst_arcs[i].link - req_edge_num;
        if (inst_arcs[i].link < 2*req_edge_num)
        {
            graph[j].link += req_edge_num;
        }
        AdMatrix[graph[j].head_node][graph[j].tail_node] = j;
        AdMatrix[graph[j].tail_node][graph[j].head_node] = j;
    }


    // find all bridges
    // if AdMatrix[i][j] = -1, (i, j) is a bridge;
    findBridge(AdMatrix);

    float rnum[10];
    // srand(seed);

    int edge_num = req_edge_num + nonreq_edge_num;
    int newnode = 0;
    int mergenode = 0;

    int nodeList[MAX_NODE_TAG_LENGTH];
    memset(nodeList, 0, sizeof(nodeList));
    for (i = 1; i <= vertex_num; i++)
    {
        nodeList[i] = 1;
    }

    // Demand change
    int demand_change;
    for (i=1; i<= req_edge_num + nonreq_edge_num; i++)
    {
        if ( (( rand() / (float) RAND_MAX) < p4) && (graph[i].change != 2))
        {
            // add some demands
            demand_change = (int)(( rand() / (double) RAND_MAX ) * graph[i].trav_cost + 1);
            if (graph[i].demand > 0) // no increase
                continue;

            if (graph[i].demand + demand_change < capacity)
            {
                graph[i].demand += demand_change; // It is might different from different solution;
            }
            else
            {
                graph[i].demand = capacity;
            }
        }
    }


    // Cost change
    int tmpNode, tmpDmd, tmpCost;
    for (i=1; i<= req_edge_num + nonreq_edge_num; i++)
    {
        for (j=0; j < 10; j++)
        {
            rnum[j] = ( rand() / (double) RAND_MAX);
            //2, 3, 7, 6, 4, 5
        }
        if(graph[i].change == 0)
        {
            // Event 1 happen
            if (rnum[2] < p2 && rnum[3] < p3 )
            {
                // road become congestion
                graph[i].trav_cost += (int)(rnum[7] * (costub - costlb));
                graph[i].change = 1;
            }
            // Event 2 happen
            if (rnum[2] < p2 && rnum[3] >= p3 )
            {
                // road break down
                if (AdMatrix[graph[i].head_node][graph[i].tail_node] != -1)
                {

                    AdMatrix[graph[i].head_node][graph[i].tail_node] = 0;
                    AdMatrix[graph[i].tail_node][graph[i].head_node] = 0;
                    findBridge(AdMatrix);

                    tmpDmd = graph[i].demand;
                    tmpCost = graph[i].trav_cost;
                    tmpNode = graph[i].tail_node;

                    newnode ++;
                    double rnum_scale = rnum[6] * 0.4 + 0.3;
                    graph[i].tail_node = vertex_num + newnode;
                    graph[i].demand = (int)(rnum_scale * tmpDmd);
                    graph[i].trav_cost = (int)(rnum_scale * tmpCost)+1;
                    graph[i].link = ++edge_num;
                    graph[i].unserved = graph[i].unserved * -1;
                    graph[i].change = 2;
                    nodeList[graph[i].tail_node] = 1;
                    


                    newnode ++;
                    graph[edge_num].head_node = vertex_num + newnode;
                    graph[edge_num].tail_node = tmpNode;
                    graph[edge_num].demand = tmpDmd - graph[i].demand;
                    graph[edge_num].trav_cost = tmpCost - graph[i].trav_cost;
                    graph[edge_num].link = i;
                    graph[edge_num].change = 2;
                    nodeList[graph[edge_num].head_node] = 1;
                }
            }
            continue;
        }

        if(graph[i].change == 1)
        {
            if (rnum[4] < p_crr) // congestion road recover
            {
                graph[i].trav_cost = cost_backup[graph[i].head_node][graph[i].tail_node];
                graph[i].change = 0;
            }
            else if (rnum[4] > p_crbb) // congestion road become worse
            {
                graph[i].trav_cost = (int) ((1.0 - rnum[8]) * graph[i].trav_cost + 1.0 * rnum[8] * cost_backup[graph[i].head_node][graph[i].tail_node]) + 1;
            }
            else // congestion road become better
            {
                graph[i].trav_cost += (int)(rnum[8] * 0.9 * (costub - costlb)) + 1;
            }
            continue;
        }

        if(graph[i].change == 2)
        {
            // breakdown road recover
            if (rnum[5] < p_bdrr)
            {
                int node1, node2, node3;
                if (graph[i].head_node < graph[i].tail_node)
                {
                    nodeList[graph[i].tail_node] = 0;
                    nodeList[graph[graph[i].link].head_node] = 0;
                    
                    node1 = graph[i].tail_node;
                    node2 = graph[graph[i].link].head_node;
                    graph[i].tail_node = graph[graph[i].link].tail_node;
                    node3 = graph[i].tail_node;
                } else {
                    nodeList[graph[i].head_node] = 0;
                    nodeList[graph[graph[i].link].tail_node] = 0;
                    
                    node1 = graph[i].head_node;
                    node2 = graph[graph[i].link].tail_node;
                    graph[i].head_node = graph[graph[i].link].head_node;
                    node3 = graph[i].head_node;
                }
                graph[i].demand += graph[graph[i].link].demand;
                graph[i].trav_cost += graph[graph[i].link].trav_cost;
                graph[i].change = 0;
                graph[graph[i].link].change = 7;
                graph[i].unserved = graph[i].unserved * -1;
                graph[graph[i].link].unserved = graph[graph[i].link].unserved * -1;
                graph[i].link = 0;
                mergenode += 2;
                // printf("To: <%d, %d>: demand: %d, cost: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].demand, graph[i].trav_cost);
                for (int k=1; k<=info->stop[0]; k++)
                {
                    if (info->stop[k] == node1 || info->stop[k] == node2)
                    {
                        info->stop[k] = node3;
                    }
                }
            }
            continue;
        }
    }


    // update graph => inst_tasks;
    Edge new_edges[MAX_TASKS_TAG_LENGTH];
    int new_vertex_num = vertex_num + newnode - mergenode;
    int new_req_edge_num = 0;
    int new_nonreq_edge_num = 0;

    int start_index = vertex_num+newnode+1;
    for (i = 1; i <= vertex_num+newnode; i++)
    {
        if (nodeList[i] == 0)
        {
            start_index = i - 1;
            break;
        }
    }
    int vnum = start_index;

    j = 0;
    int head, tail;
    for (i = 1; i <= edge_num; i++)
    {
        if (graph[i].change != 7)
        {
            j++;
            if (graph[i].tail_node > start_index)
            {
                head = graph[i].head_node;
                tail = ++vnum;
                // graph[graph[i].link].link = j;

            } else if (graph[i].head_node > start_index)
            {
                head = ++vnum;
                tail = graph[i].tail_node;
                new_edges[j].link = graph[i].link;
                // new_edges[graph[i].link].link = j;
            } else {
                head = graph[i].head_node;
                tail = graph[i].tail_node;
                new_edges[j].link = graph[i].link;
            }

            if (graph[i].link > i)
            {
                graph[graph[i].link].link = j;
            }
            if (graph[i].link < i)
            {
                new_edges[graph[i].link].link = j;
                new_edges[j].link = graph[i].link;
            }

            new_edges[j].head_node = head;
            new_edges[j].tail_node = tail;
            new_edges[j].trav_cost = graph[i].trav_cost;
            new_edges[j].demand = graph[i].demand;
            new_edges[j].change = graph[i].change;
            new_edges[j].unserved = graph[i].unserved;
            if (new_edges[j].demand > 0)
            {
                new_req_edge_num ++;
            } else{
                new_nonreq_edge_num ++;
            }
        }
    }

//    saveGraph(new_edges, *info, vnum, new_req_edge_num, new_nonreq_edge_num, j);

    int new_task_num = 2 * new_req_edge_num;
    Task new_inst_tasks[MAX_TASKS_TAG_LENGTH];
    Arc new_inst_arcs[MAX_ARCS_TAG_LENGTH];

    int mapping1[2*new_req_edge_num+new_nonreq_edge_num+1];
    int mapping2[2*new_req_edge_num+new_nonreq_edge_num+1];
    memset(mapping1, 0, sizeof(mapping1)); // mapping1[u] = i -> the uth edge in new_inst_tasks -> the ith edge in new_edges 
    memset(mapping2, 0, sizeof(mapping2)); // mapping2[i] = u -> the ith edge in new_edges -> the uth edge in new_inst_tasks
    int u = 0, v = new_task_num;
    for (i=1; i <= j; i++)
    {
        if (new_edges[i].demand > 0)
        {
            u ++;
            new_inst_tasks[u].head_node = new_edges[i].head_node;
            new_inst_tasks[u].tail_node = new_edges[i].tail_node;
            new_inst_tasks[u].serv_cost = new_edges[i].trav_cost;
            if (new_inst_tasks[u].serv_cost == 0)
            {
                new_inst_tasks[u].serv_cost = 10;
            }
            new_inst_tasks[u].dead_cost = new_edges[i].trav_cost;
            new_inst_tasks[u].demand = new_edges[i].demand;
            new_inst_tasks[u].inverse = u + new_req_edge_num;

            new_inst_tasks[u+new_req_edge_num].head_node =  new_inst_tasks[u].tail_node;
            new_inst_tasks[u+new_req_edge_num].tail_node = new_inst_tasks[u].head_node;
            new_inst_tasks[u+new_req_edge_num].dead_cost = new_inst_tasks[u].dead_cost;
            new_inst_tasks[u+new_req_edge_num].serv_cost = new_inst_tasks[u].serv_cost;
            new_inst_tasks[u+new_req_edge_num].demand = new_inst_tasks[u].demand;
            new_inst_tasks[u+new_req_edge_num].inverse = u;

            new_inst_arcs[u].head_node = new_inst_tasks[u].head_node;
            new_inst_arcs[u].tail_node = new_inst_tasks[u].tail_node;
            new_inst_arcs[u].trav_cost = new_inst_tasks[u].dead_cost;
            new_inst_arcs[u].change = new_edges[i].change;
            new_inst_arcs[u].link = new_edges[i].link;
            mapping1[u] = i;
            mapping2[i] = u;

            new_inst_arcs[u+new_req_edge_num].head_node = new_inst_tasks[u].tail_node;
            new_inst_arcs[u+new_req_edge_num].tail_node = new_inst_tasks[u].head_node;
            new_inst_arcs[u+new_req_edge_num].trav_cost = new_inst_tasks[u].dead_cost;
            new_inst_arcs[u+new_req_edge_num].change = new_inst_arcs[u].change;

        } else{
            v++;
            new_inst_arcs[v].head_node = new_edges[i].head_node;
            new_inst_arcs[v].tail_node = new_edges[i].tail_node;
            new_inst_arcs[v].trav_cost = new_edges[i].trav_cost;
            new_inst_arcs[v].change = new_edges[i].change;
            new_inst_arcs[v].link = new_edges[i].link;
            mapping1[v] = i;
            mapping2[i] = v;


            new_inst_arcs[v+new_nonreq_edge_num].head_node = new_edges[i].tail_node;
            new_inst_arcs[v+new_nonreq_edge_num].tail_node = new_edges[i].head_node;
            new_inst_arcs[v+new_nonreq_edge_num].trav_cost = new_edges[i].trav_cost;
            new_inst_arcs[v+new_nonreq_edge_num].change = new_edges[i].change;
        }
    }

    // mapping unserved tasks
    int tmp_new_seq[MAX_TASK_SEQ_LENGTH];
    memcpy(tmp_new_seq, unserved_seq, sizeof(tmp_new_seq));
    for (i = 1; i <= j; i++)
    {
        int idx;
        if (new_edges[i].unserved != 0)
        {
            if (new_edges[i].unserved > 10000)
            {
                idx = new_edges[i].unserved - 10000;
                if (mapping2[i] > new_req_edge_num)
                {
                    tmp_new_seq[idx] = mapping2[i]-new_req_edge_num;
                } else
                {
                    tmp_new_seq[idx] = mapping2[i]+new_req_edge_num;
                }
                continue;
            }

            if (new_edges[i].unserved < 0)
            {
                if (new_edges[i].unserved < -10000)
                {
                    idx = new_edges[i].unserved * -1 - 10000;
                } else {
                    idx = new_edges[i].unserved * -1;
                }
                tmp_new_seq[idx] = -1;
                continue;
            }

            idx = new_edges[i].unserved;
            tmp_new_seq[idx] = mapping2[i];
        }
    }
    for (i = 1; i <= tmp_new_seq[0]; i++)
    {
        if (tmp_new_seq[i] > 2*new_req_edge_num)
        {
            delete_element(tmp_new_seq, i);
        }
    }
    memcpy(info->remain_seqs, tmp_new_seq, sizeof(tmp_new_seq));



    int temp;
    for(i = 1; i <= new_req_edge_num; i++)
    {
        if(new_inst_arcs[i].change == 2)
        {
            new_inst_arcs[i].link = mapping2[new_edges[mapping1[i]].link];
        }
    }
    for(i = new_task_num + 1; i <= new_task_num + new_nonreq_edge_num; i++)
    {
        if(new_inst_arcs[i].change == 2)
        {
            new_inst_arcs[i].link = mapping2[new_edges[mapping1[i]].link];
        }
    }

    req_edge_num = new_req_edge_num;
    nonreq_edge_num = new_nonreq_edge_num;
    task_num = new_task_num;
    vertex_num = new_vertex_num;
    total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
    memcpy(inst_tasks, new_inst_tasks, sizeof(new_inst_tasks));
    memcpy(inst_arcs, new_inst_arcs, sizeof(new_inst_arcs));


    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;

    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;

}


void saveGraph(Edge *graph, Vehicles info, int vnum, int new_req_edge_num, int new_nonreq_edge_num, int edge_num)
{
    int i;
    char path[30];
    static int num = 0;

    num ++;
    sprintf(path, "../dmap/egl/%d", num);
    if (num == 3)
    {
        num = 0;
    }
//    printf("%s\n", path);
    FILE *fp;
    fp = fopen(path, "w+");
    // fprintf(fp, "name: %s\n", map);
    // fprintf(fp, "vertices: %d\n", vnum);
    // fprintf(fp, "req_edge_num: %d\n", new_req_edge_num);
    // fprintf(fp, "non_req_edge_num: %d\n", new_nonreq_edge_num);
    // fprintf(fp, "vehicles: %d\n", vehicle_num);
    // fprintf(fp, "outside_vehicles: %d\n", info.stop[0]);
    // fprintf(fp, "stop_point: ");
    // for (i = 1; i <= info.stop[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.stop[i]);
    // }
    // fprintf(fp, "\n");

    // fprintf(fp, "capacity: %d\n", capacity);
    // fprintf(fp, "remaining_capacity: ");
    // for (i = 1; i <= info.remain_capacity[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.remain_capacity[i]);
    // }
    // fprintf(fp, "\n");
    // fprintf(fp, "remaining_tasks: ");
    // for (i = 0; i <= info.remain_seqs[0]; i++)
    // {
    //     fprintf(fp, "%d ", info.remain_seqs[i]);
    // }
    // fprintf(fp, "\nEdges: \n");
    // for (i = 1; i <= edge_num; i++)
    // {
    //     fprintf(fp, "(%d, %d), cost: %d, demand: %d, change: %d\n", graph[i].head_node, graph[i].tail_node, graph[i].trav_cost, graph[i].demand, graph[i].change);
    // }

    for (i = 1; i <= edge_num; i++)
    {
        fprintf(fp, "%d %d %d\n", graph[i].head_node, graph[i].tail_node, graph[i].trav_cost);
    }
    fclose(fp);
}

void findBridge(int (*AdMatrix)[MAX_NODE_TAG_LENGTH])
{
    int visited[vertex_num+1];
    int disc[vertex_num+1];
    int low[vertex_num+1];
    int parent[vertex_num+1];

    memset(visited, 0, sizeof(visited));
    memset(disc, INF, sizeof(disc));
    memset(low, INF, sizeof(low));
    memset(parent, -1, sizeof(parent));

    int time = 0;
    for (int i=1; i <= vertex_num; i++)
    {
        if ( !visited[i])
            dfs(i, visited, parent, low, disc, &time, AdMatrix);
    }

}

void dfs(int u, int *visited, int *parent, int *low, int *disc, int *time, int (*AdMatrix)[MAX_NODE_TAG_LENGTH])
{
    visited[u] = 1;
    disc[u] = *time;
    low[u] = *time;
    *time  += 1;

    int v;
    for (int i=1; i <= vertex_num; i++)
    {
        if (AdMatrix[u][i] > 0)
        {
            v = i;
            if ( !visited[v] )
            {
                parent[v] = u;
                dfs(v, visited, parent, low, disc, time, AdMatrix);

                low[u] = (low[u] < low[v] )? low[u] : low[v];
                if (low[v] > disc[u])
                {
                    AdMatrix[u][v] = -1;
                    AdMatrix[v][u] = -1;
                //    printf("bridge: %d, %d\n", u, v); //get bridges
                }
            }
            else if ( v != parent[u])
            {
                low[u] = (low[u] < disc[v] )? low[u] : disc[v];
            }
        }

    }


}