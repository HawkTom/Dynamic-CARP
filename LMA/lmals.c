//
// Created by hao on 22/06/2020.
//


# include "LMA.h"

int lma_single_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_double_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int x, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_swap(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);
int lma_two_opt(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks);

void detect_solution(Individual *mted_child, lns_route *curr_solution, const Task *inst_tasks);
void showRoute(int *route1, int *route2);

void detect_solution(Individual *mted_child, lns_route *curr_solution, const Task *inst_tasks)
{
    mted_child->Sequence[0] = 1;
    int k = 0;
    for (int i = 1; i <= curr_solution->Route[0][0]; i++)
    {
        if (curr_solution->Route[i][0] > 2)
        {
            mted_child->Sequence[0] --;
            JoinArray(mted_child->Sequence, curr_solution->Route[i]);
            k ++;
            mted_child->Loads[k] = curr_solution->loads[i];
        }
    }
    mted_child->Loads[0] = k;

    if ( curr_solution->total_cost != get_task_seq_total_cost(mted_child->Sequence, inst_tasks))
        printf("total cost wrong \n");

    if (curr_solution->total_vio_loads != get_total_vio_load(mted_child->Loads))
        printf("total vio load wrong \n");
}

void showRoute(int *route1, int *route2)
{
    printf("%d \t", route1[0]);
    for (int i = 1; i <= route1[0]; i++)
    {
        printf("%d \t", route1[i]);
    }
    printf("\n%d \t", route2[0]);

    for (int i = 1; i <= route2[0]; i++)
    {
        printf("%d \t", route2[i]);
    }
    printf("\n\n");
}

int check_cost1(lns_route curr_solution, const Task *inst_tasks);
int check_cost1(lns_route curr_solution, const Task *inst_tasks)
{
    int i;
    int cost = 0, tmp_cost;
    for (i = 1; i <= curr_solution.Route[0][0]; i++)
    {
        tmp_cost = get_task_seq_total_cost(curr_solution.Route[i], inst_tasks);
        printf("%d \t", tmp_cost);
        cost += tmp_cost;
    }
    printf("\n");
    if (cost != curr_solution.total_cost)
    {
        printf("cost error. \n");
        return 0;
    }
    return 1;
}


void lma_lns(Individual *indi, Individual *mted_child, const Task *inst_tasks)
{

    int i, j, k;


    lns_route curr_solution, best_fsb_solution, next_solution;

    int Positions[101];
    find_ele_positions(Positions, indi->Sequence, 0);
    curr_solution.Route[0][0] = Positions[0] - 1;
    curr_solution.loads[0] = Positions[0] - 1;
    for (i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(indi->Sequence, Positions[i], Positions[i+1], curr_solution.Route[i]); // Route[i]: 0 x x x x 0
        curr_solution.loads[i] = 0;
        for (j = Positions[i]+1; j < Positions[i+1]; j++)
        {
            curr_solution.loads[i] += inst_tasks[indi->Sequence[j]].demand;
        }

    }
    curr_solution.fitness = indi->TotalCost + LAMBDA * indi->TotalVioLoad;;
    curr_solution.total_vio_loads = indi->TotalVioLoad;
    curr_solution.total_cost = indi->TotalCost;

    int u, pos_u, trip_u, v, pos_v, trip_v, x, y, inv_u;
    int improve = 0;

    next_solution = curr_solution;
    while (1)
    {
        for (trip_u = 1; trip_u <= curr_solution.Route[0][0]; trip_u ++)
        {
            for (pos_u = 2; pos_u < curr_solution.Route[trip_u][0]; pos_u++)
            {
                u = curr_solution.Route[trip_u][pos_u];
                inv_u = inst_tasks[u].inverse;

                next_solution.total_cost = curr_solution.total_cost
                                           - min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                           - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node]
                                           + min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[inv_u].head_node]
                                           + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node];


                // bug: inverse error;
                // next_solution.total_cost = curr_solution.total_cost
                //                            - min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                //                            - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node]
                //                            + min_cost[inst_tasks[curr_solution.Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].tail_node]
                //                            + min_cost[inst_tasks[u].head_node][inst_tasks[curr_solution.Route[trip_u][pos_u+1]].head_node];

                next_solution.fitness = next_solution.total_cost + LAMBDA * next_solution.total_vio_loads;
                if (next_solution.fitness < curr_solution.fitness)
                {
                    // printf("++++");
                    // check_cost1(curr_solution, inst_tasks);
                    curr_solution.total_cost = next_solution.total_cost;
                    curr_solution.fitness = next_solution.fitness;
                    curr_solution.Route[trip_u][pos_u] = inst_tasks[u].inverse;
                    // check_cost1(curr_solution, inst_tasks);
                }
                u = curr_solution.Route[trip_u][pos_u];
                

                for (trip_v = trip_u; trip_v <= curr_solution.Route[0][0]; trip_v ++)
                {
                    for (pos_v = pos_u + 1; pos_v < curr_solution.Route[trip_v][0]; pos_v ++)
                    {

                        v = curr_solution.Route[trip_v][pos_v];

//                        printf("\n\n\nu: %d, v: %d, trip_u: %d, trip_v: %d, pos_u: %d, pos_v: %d\n", u, v, trip_u, trip_v, pos_u, pos_v);

                    
                        improve = lma_single_insertion(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                        
                        // check_cost1(curr_solution, inst_tasks);
//                        detect_solution(mted_child, &curr_solution, inst_tasks);

                        if (improve){
//                            printf("single insertion \n");
                            goto new_step;
                        }

                        if (trip_u == trip_v && pos_v - pos_u > 1 || trip_v != trip_u)
                        {

                            x = curr_solution.Route[trip_u][pos_u+1];
                            if (trip_v != trip_u && x != 0)
                            {
                                improve = lma_double_insertion(&curr_solution, &next_solution, u, x, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                                // check_cost1(curr_solution, inst_tasks);
                                if (improve){
//                                    printf("double insertion \n");
                                    goto new_step;
                                }
                            }



                            improve = lma_swap(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
                            // check_cost1(curr_solution, inst_tasks);
                            if (improve){
//                                printf("swap \n");
                                goto new_step;
                            }

                            // *********
                            // Two-opt is not suitable for directed CARP.
                            // ********
//                             y = curr_solution.Route[trip_v][pos_v+1];
//                             if (trip_v != trip_u && x != 0 && y != 0)
//                             {
//                                 improve = lma_two_opt(&curr_solution, &next_solution, u, v, trip_u, trip_v, pos_u, pos_v, inst_tasks);
//                                 // check_cost1(curr_solution, inst_tasks);
//                                 if (improve){
// //                                    printf("two-opt \n");
//                                     goto new_step;
//                                 }
//                             }

                        }
                    }
                }
            }
        }


    new_step:
        if (improve)
        {
            continue;
        } else {
            break;
        }
    }

    // route -> sequence
    mted_child->Sequence[0] = 1;
    k = 0;
    for (i = 1; i <= curr_solution.Route[0][0]; i++)
    {
        if (curr_solution.Route[i][0] > 2)
        {
            mted_child->Sequence[0] --;
            JoinArray(mted_child->Sequence, curr_solution.Route[i]);
            k ++;
            mted_child->Loads[k] = curr_solution.loads[i];
        }
    }
    mted_child->Loads[0] = k;
    mted_child->TotalCost = curr_solution.total_cost;
    mted_child->TotalVioLoad = curr_solution.total_vio_loads;

    if ( mted_child->TotalCost != get_task_seq_total_cost(mted_child->Sequence, inst_tasks))
    {
        printf("lmsals total cost error \n");
        // longjmp(buf, 2);
        exit(0);
    }

    // if (mted_child->TotalVioLoad != 0)
    // {
    //     printf("lmsla load error\n");
    //     longjmp(buf, 2);
    //     exit(0);
    // }

}

void check_route_cost(const Individual Solution, const Task *inst_tasks)
{
    int i, j;
    int route[250];
    for (i = 1; i <= Solution.Sequence[0]; i++)
    {
        if (Solution.Sequence[i] == 0)
        {
            if (i > 1)
            {
                int cost = get_task_seq_total_cost(route, inst_tasks);
                
            }
            memset(route, 0, sizeof(route));
        } else
        {
            route[0] ++;
            route[route[0]] = Solution.Sequence[i];
        }
        
    }
}

int lma_single_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks)
{
    next_solution->loads[trip_u] -= inst_tasks[u].demand;
    next_solution->loads[trip_v] += inst_tasks[u].demand;


    if (curr_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_u] - capacity;
    if (curr_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_v] - capacity;

    if (next_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_u] - capacity;
    if (next_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_v] - capacity;



    int flag;
    if (pos_v == 2) // first task
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[v].head_node];

    }
    else
    {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }

    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        // printf("-----\n");
        // check_cost1(*curr_solution, inst_tasks);
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];

        if (flag) {
            add_element(curr_solution->Route[trip_v], u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], u, pos_v+1);
        }
        delete_element(curr_solution->Route[trip_u], pos_u);
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }

    int inv_u = inst_tasks[u].inverse;
    if (pos_v == 2) // first task
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[v].head_node];

    }
    else
    {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }
    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;

    if (next_solution->fitness < curr_solution->fitness)
    {
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];

        if (flag) {
            add_element(curr_solution->Route[trip_v], inv_u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], inv_u, pos_v + 1);
        }
        delete_element(curr_solution->Route[trip_u], pos_u);
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }


    next_solution->total_cost = curr_solution->total_cost;
    next_solution->total_vio_loads = curr_solution->total_vio_loads;
    next_solution->fitness = curr_solution->fitness;
    next_solution->loads[trip_u] = curr_solution->loads[trip_u];
    next_solution->loads[trip_v] = curr_solution->loads[trip_v];
    return 0;
}


int lma_double_insertion(lns_route *curr_solution, lns_route *next_solution, int u, int x, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks)
{

    next_solution->loads[trip_u] -= inst_tasks[u].demand + inst_tasks[x].demand;
    next_solution->loads[trip_v] += inst_tasks[u].demand + inst_tasks[x].demand;

    if (curr_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_u] - capacity;
    if (curr_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_v] - capacity;

    if (next_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_u] - capacity;
    if (next_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_v] - capacity;

    int flag;
    if (pos_v == 2)
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    + min_cost[inst_tasks[x].tail_node][inst_tasks[v].head_node];
    } else {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    + min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }
    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        if (flag) {
            add_element(curr_solution->Route[trip_v], x, pos_v);
            add_element(curr_solution->Route[trip_v], u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], u, pos_v + 1);
            add_element(curr_solution->Route[trip_v], x, pos_v + 2);
        }

        delete_element(curr_solution->Route[trip_u], pos_u+1);
        delete_element(curr_solution->Route[trip_u], pos_u);
        return 1;
    }

    int inv_u = inst_tasks[u].inverse;
    if (pos_v == 2)
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[x].head_node]
                                    + min_cost[inst_tasks[x].tail_node][inst_tasks[v].head_node];
    } else {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[x].head_node]
                                    + min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }
    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        if (flag) {
            add_element(curr_solution->Route[trip_v], x, pos_v);
            add_element(curr_solution->Route[trip_v], inv_u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], inv_u, pos_v + 1);
            add_element(curr_solution->Route[trip_v], x, pos_v + 2);
        }

        delete_element(curr_solution->Route[trip_u], pos_u+1);
        delete_element(curr_solution->Route[trip_u], pos_u);
        return 1;
    }


    int inv_x = inst_tasks[x].inverse;
    if (pos_v == 2)
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[inv_x].head_node]
                                    + min_cost[inst_tasks[inv_x].tail_node][inst_tasks[v].head_node];
    } else {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[u].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[inv_x].head_node]
                                    + min_cost[inst_tasks[inv_x].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }
    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        if (flag) {
            add_element(curr_solution->Route[trip_v], inv_x, pos_v);
            add_element(curr_solution->Route[trip_v], u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], u, pos_v + 1);
            add_element(curr_solution->Route[trip_v], inv_x, pos_v + 2);
        }

        delete_element(curr_solution->Route[trip_u], pos_u+1);
        delete_element(curr_solution->Route[trip_u], pos_u);
        return 1;
    }

    if (pos_v == 2)
    {
        flag = 1;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[inv_x].head_node]
                                    + min_cost[inst_tasks[inv_x].tail_node][inst_tasks[v].head_node];
    } else {
        flag = 0;
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[x].head_node]
                                    - min_cost[inst_tasks[x].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+2]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[inv_u].head_node]
                                    + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[inv_x].head_node]
                                    + min_cost[inst_tasks[inv_x].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];
    }
    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        if (flag) {
            add_element(curr_solution->Route[trip_v], inv_x, pos_v);
            add_element(curr_solution->Route[trip_v], inv_u, pos_v);
        } else {
            add_element(curr_solution->Route[trip_v], inv_u, pos_v + 1);
            add_element(curr_solution->Route[trip_v], inv_x, pos_v + 2);
        }

        delete_element(curr_solution->Route[trip_u], pos_u+1);
        delete_element(curr_solution->Route[trip_u], pos_u);
        return 1;
    }

    next_solution->total_cost = curr_solution->total_cost;
    next_solution->total_vio_loads = curr_solution->total_vio_loads;
    next_solution->fitness = curr_solution->fitness;
    next_solution->loads[trip_u] = curr_solution->loads[trip_u];
    next_solution->loads[trip_v] = curr_solution->loads[trip_v];
    return 0;
}

int lma_swap(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks)
{

    next_solution->loads[trip_u] -= inst_tasks[u].demand - inst_tasks[v].demand;
    next_solution->loads[trip_v] += inst_tasks[u].demand - inst_tasks[v].demand;

    if (curr_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_u] - capacity;
    if (curr_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads -= curr_solution->loads[trip_v] - capacity;

    if (next_solution->loads[trip_u] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_u] - capacity;
    if (next_solution->loads[trip_v] > capacity)
        next_solution->total_vio_loads += next_solution->loads[trip_v] - capacity;

    next_solution->total_cost = curr_solution->total_cost
                                - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[v].head_node]
                                + min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[u].head_node]
                                + min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];




    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        // check_cost1(*curr_solution, inst_tasks);
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        curr_solution->Route[trip_u][pos_u] = v;
        curr_solution->Route[trip_v][pos_v] = u;
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }

    int inv_u = inst_tasks[u].inverse;
    next_solution->total_cost = curr_solution->total_cost
                                - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[v].head_node]
                                + min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[inv_u].head_node]
                                + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];


    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        // check_cost1(*curr_solution, inst_tasks);
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        curr_solution->Route[trip_u][pos_u] = v;
        curr_solution->Route[trip_v][pos_v] = inv_u;
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }


    int inv_v = inst_tasks[v].inverse;
    next_solution->total_cost = curr_solution->total_cost
                                - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[inv_v].head_node]
                                + min_cost[inst_tasks[inv_v].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[u].head_node]
                                + min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];

    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        // check_cost1(*curr_solution, inst_tasks);
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        curr_solution->Route[trip_u][pos_u] = inv_v;
        curr_solution->Route[trip_v][pos_v] = u;
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }

    next_solution->total_cost = curr_solution->total_cost
                                - min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[u].head_node]
                                - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u-1]].tail_node][inst_tasks[inv_v].head_node]
                                + min_cost[inst_tasks[inv_v].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                - min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[v].head_node]
                                - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                + min_cost[inst_tasks[curr_solution->Route[trip_v][pos_v-1]].tail_node][inst_tasks[inv_u].head_node]
                                + min_cost[inst_tasks[inv_u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];

    next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;
    if (next_solution->fitness < curr_solution->fitness)
    {
        // check_cost1(*curr_solution, inst_tasks);
        curr_solution->total_cost = next_solution->total_cost;
        curr_solution->total_vio_loads = next_solution->total_vio_loads;
        curr_solution->fitness = next_solution->fitness;
        curr_solution->loads[trip_u] = next_solution->loads[trip_u];
        curr_solution->loads[trip_v] = next_solution->loads[trip_v];
        curr_solution->Route[trip_u][pos_u] = inv_v;
        curr_solution->Route[trip_v][pos_v] = inv_u;
        // check_cost1(*curr_solution, inst_tasks);
        return 1;
    }

    next_solution->total_cost = curr_solution->total_cost;
    next_solution->total_vio_loads = curr_solution->total_vio_loads;
    next_solution->fitness = curr_solution->fitness;
    next_solution->loads[trip_u] = curr_solution->loads[trip_u];
    next_solution->loads[trip_v] = curr_solution->loads[trip_v];
    return 0;
}


int lma_two_opt(lns_route *curr_solution, lns_route *next_solution, int u, int v, int trip_u, int trip_v, int pos_u, int pos_v, const Task *inst_tasks)
{
    if (trip_u == trip_v)
    {
        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[v].tail_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];

        next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;

        if (next_solution->fitness < curr_solution->fitness)
        {
            curr_solution->total_cost = next_solution->total_cost;
            curr_solution->total_vio_loads = next_solution->total_vio_loads;
            curr_solution->fitness = next_solution->fitness;
            for (int i = pos_u + 1; i <= pos_v; i++)
            {
                curr_solution->Route[trip_u][i] = inst_tasks[curr_solution->Route[trip_u][i]].inverse;
            }
            ReverseDirection(curr_solution->Route[trip_u], pos_u+1, pos_v);
            // check_cost1(*curr_solution, inst_tasks);


            return 1;
        }

    } else {


        int load_seg1 = 0, load_seg2 = 0;
        for (int i = 2; i <= pos_u; i++)
        {
            load_seg1 += inst_tasks[curr_solution->Route[trip_u][i]].demand;
        }
        for (int i = 2; i <= pos_v; i++)
        {
            load_seg2 += inst_tasks[curr_solution->Route[trip_v][i]].demand;
        }

        // case 1
        next_solution->loads[trip_u] = load_seg1 + curr_solution->loads[trip_v] - load_seg2;
        next_solution->loads[trip_v] = load_seg2 + curr_solution->loads[trip_u] - load_seg1;

        if (curr_solution->loads[trip_u] > capacity)
            next_solution->total_vio_loads -= curr_solution->loads[trip_u] - capacity;
        if (curr_solution->loads[trip_v] > capacity)
            next_solution->total_vio_loads -= curr_solution->loads[trip_v] - capacity;

        if (next_solution->loads[trip_u] > capacity)
            next_solution->total_vio_loads += next_solution->loads[trip_u] - capacity;
        if (next_solution->loads[trip_v] > capacity)
            next_solution->total_vio_loads += next_solution->loads[trip_v] - capacity;

        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node];

        next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;

        if (next_solution->fitness < curr_solution->fitness)
        {
//            printf("case1 \n");
            curr_solution->total_cost = next_solution->total_cost;
            curr_solution->total_vio_loads = next_solution->total_vio_loads;
            curr_solution->fitness = next_solution->fitness;
            curr_solution->loads[trip_u] = next_solution->loads[trip_u];
            curr_solution->loads[trip_v] = next_solution->loads[trip_v];

            int tmp_route[250];
            memcpy(tmp_route, curr_solution->Route[trip_u], sizeof(curr_solution->Route[trip_u]));
            curr_solution->Route[trip_u][0] = pos_u;
            for (int i = pos_v + 1; i <= curr_solution->Route[trip_v][0]; i++)
            {
                curr_solution->Route[trip_u][0] ++;
                curr_solution->Route[trip_u][curr_solution->Route[trip_u][0]] = curr_solution->Route[trip_v][i];
            }

            curr_solution->Route[trip_v][0] = pos_v;
            for (int i = pos_u + 1; i <= tmp_route[0]; i++)
            {
                curr_solution->Route[trip_v][0] ++;
                curr_solution->Route[trip_v][curr_solution->Route[trip_v][0]] = tmp_route[i];
            }
            // check_cost1(*curr_solution, inst_tasks);
            return 1;
        }

        // case 2
        if ( inst_tasks[curr_solution->Route[trip_u][2]].vt > 0 && inst_tasks[curr_solution->Route[trip_v][2]].vt > 0)
        {
            goto out;
        }

        next_solution->loads[trip_u] = load_seg1 + load_seg2;
        next_solution->loads[trip_v] = curr_solution->loads[trip_u] - load_seg1 + curr_solution->loads[trip_v] - load_seg2 ;

        if (curr_solution->loads[trip_u] > capacity)
            next_solution->total_vio_loads -= curr_solution->loads[trip_u] - capacity;
        if (curr_solution->loads[trip_v] > capacity)
            next_solution->total_vio_loads -= curr_solution->loads[trip_v] - capacity;

        if (next_solution->loads[trip_u] > capacity)
            next_solution->total_vio_loads += next_solution->loads[trip_u] - capacity;
        if (next_solution->loads[trip_v] > capacity)
            next_solution->total_vio_loads += next_solution->loads[trip_v] - capacity;

        next_solution->total_cost = curr_solution->total_cost
                                    - min_cost[inst_tasks[u].tail_node][inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node]
                                    - min_cost[inst_tasks[v].tail_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node]
                                    + min_cost[inst_tasks[u].tail_node][inst_tasks[v].tail_node]
                                    + min_cost[inst_tasks[curr_solution->Route[trip_u][pos_u+1]].head_node][inst_tasks[curr_solution->Route[trip_v][pos_v+1]].head_node];

        next_solution->fitness = next_solution->total_cost + LAMBDA * next_solution->total_vio_loads;

        if (next_solution->fitness < curr_solution->fitness)
        {
        //    printf("case2 \n");
            printf("before : %d\n", curr_solution->total_cost);
            // showRoute(curr_solution->Route[trip_u], curr_solution->Route[trip_v]);
            // check_cost1(*curr_solution, inst_tasks);
            curr_solution->total_cost = next_solution->total_cost;
            curr_solution->total_vio_loads = next_solution->total_vio_loads;
            curr_solution->fitness = next_solution->fitness;
            curr_solution->loads[trip_u] = next_solution->loads[trip_u];
            curr_solution->loads[trip_v] = next_solution->loads[trip_v];

            int tmp_route2[250], tmp_route1[250];

            memcpy(tmp_route1, curr_solution->Route[trip_u], sizeof(curr_solution->Route[trip_u]));
            memcpy(tmp_route2, curr_solution->Route[trip_v], sizeof(curr_solution->Route[trip_v]));
            
            

            curr_solution->Route[trip_u][0] = pos_u;
            for (int i = pos_v; i >= 1; i--)
            {
                curr_solution->Route[trip_u][0] ++;
                curr_solution->Route[trip_u][curr_solution->Route[trip_u][0]] = inst_tasks[tmp_route2[i]].inverse;
            }

            curr_solution->Route[trip_v][0] = 0;
            for (int i = tmp_route1[0]; i >= pos_u + 1; i--)
            {
                curr_solution->Route[trip_v][0] ++;
                curr_solution->Route[trip_v][curr_solution->Route[trip_v][0]] = inst_tasks[tmp_route1[i]].inverse;
            }
            for (int i = pos_v + 1; i <= tmp_route2[0]; i++)
            {
                curr_solution->Route[trip_v][0] ++;
                curr_solution->Route[trip_v][curr_solution->Route[trip_v][0]] = tmp_route2[i];
            }

            if ( inst_tasks[curr_solution->Route[trip_u][curr_solution->Route[trip_u][0]-1]].vt > 0)
            {
                int tmp_route3[250];
                int route_tasks = curr_solution->Route[trip_u][0];
                memset(tmp_route3, 0, sizeof(tmp_route3));
                tmp_route3[0] = route_tasks;
                for (int i = 2; i < route_tasks; i++)
                {
                    tmp_route3[i] = inst_tasks[curr_solution->Route[trip_u][route_tasks+1-i]].inverse;
                }
                memcpy(curr_solution->Route[trip_u], tmp_route3, sizeof(tmp_route3));
            }

            printf("later: %d\n", curr_solution->total_cost);
            // showRoute(curr_solution->Route[trip_u], curr_solution->Route[trip_v]);
            // check_cost1(*curr_solution, inst_tasks);
            return 1;
        }
    }

    out:
    next_solution->total_cost = curr_solution->total_cost;
    next_solution->total_vio_loads = curr_solution->total_vio_loads;
    next_solution->fitness = curr_solution->fitness;
    next_solution->loads[trip_u] = curr_solution->loads[trip_u];
    next_solution->loads[trip_v] = curr_solution->loads[trip_v];
    return 0;
}



