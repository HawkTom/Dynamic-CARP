//
// Created by hao on 27/07/2020.
//

#include "MASDC.h"


void roulette_wheel_selection(const int *fitness, int *choose)
{
    int i;
    int pro[300], total=0;
    memset(pro, 0, sizeof(pro));
    for (i = 1; i <= fitness[0]; i++)
    {
        pro[i] = pro[i-1] + fitness[i];
        total += fitness[i];
    }

    if(total != pro[fitness[0]])
    {
        printf("route wheel selection wrong\n");
        exit(0);
    }

    int rnum1 = (int)((rand() / (double) RAND_MAX) * total);

    for (i = 1; i <= fitness[0]; i++)
    {
        if (pro[i] > rnum1)
        {
            choose[1] = i;
            break;
        }
    }

    int rnum2 = (int)((rand() / (double) RAND_MAX) * total);

    for (i = 1; i <= fitness[0]; i++)
    {
        if (pro[i] > rnum2)
        {
            choose[2] = i;
            if (choose[2] == choose[1] && choose[2] < fitness[0])
            {
                choose[2] ++;
            }
            if (choose[2] == choose[1] && choose[2] == fitness[0])
            {
                choose[2] --;
            }
            break;
        }
    }
}


int MASDC_local_search(Individual *indi, Individual *child, const int *stop, const int *remain_capacity, const Task *inst_tasks, const int max_fe)
{
    int i, j, k;
    int fe = 0;

    int new_seq[MAX_TASK_SEQ_LENGTH], best_seq[MAX_TASK_SEQ_LENGTH];
    int fitness, best_fitness = INF;
    int best_split[MAX_TASK_SEQ_LENGTH], new_split[MAX_TASK_SEQ_LENGTH];
    int task1, task2;
    // single Insertion;
    for (i = 1; i < indi->Assignment[0]; i++)
    {
        task1 = indi->Assignment[i];
        memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
        delete_element(new_seq, i);
        for (j = i+1; j <= indi->Assignment[0]; j++)
        {
            add_element(new_seq, task1, j);

            fitness = MASDC_split(new_split, new_seq, stop, remain_capacity, inst_tasks);
            fe += 3;
            if (fitness < best_fitness)
            {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
            }
            delete_element(new_seq, j);
        }
    }
    // double insertion
    for (i = 1; i < indi->Assignment[0]-1; i++)
    {
        task1 = indi->Assignment[i];
        task2 = indi->Assignment[i+1];
        memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
        for (k = i; k < new_seq[0]-1; k++)
        {
            new_seq[k] = new_seq[k+2];
        }

        for (j = 1; j <= indi->Assignment[0]-2; j++)
        {
            // can be optimized here
            for (k = new_seq[0]; k > j+1; k --)
            {
                new_seq[k] = new_seq[k-2];
            }
            new_seq[j] = task1;
            new_seq[j+1] = task2;

            fitness = MASDC_split(new_split, new_seq, stop, remain_capacity, inst_tasks);
            fe += 3;
            if (fitness < best_fitness)
            {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
            }

            for (k = j; k < new_seq[0]-1; k++)
            {
                new_seq[k] = new_seq[k+2];
            }

        }
    }

    // swap
    memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
    for (i = 1; i < indi->Assignment[0]; i++)
    {
        for (j = i + 1; j <= indi->Assignment[0]; j++)
        {
            int tmp = new_seq[i];
            new_seq[i] = new_seq[j];
            new_seq[j] = tmp;
            fitness = MASDC_split(new_split, new_seq, stop, remain_capacity, inst_tasks);
            fe += 3;
            if (fitness < best_fitness)
            {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
            }
            tmp = new_seq[i];
            new_seq[i] = new_seq[j];
            new_seq[j] = tmp;
        }
    }

    memcpy(child->Sequence, best_split, sizeof(best_split));
    memcpy(child->Assignment, best_seq, sizeof(best_seq));
    child->TotalCost = best_fitness;
    return fe;
}


void MASDC_crossover(const int *parent1, const int *parent2, int *child)
{
    int i, j, tmp;
    int nsize = parent1[0];
    int key1[MAX_TASK_TAG_LENGTH];
    int key2[MAX_TASK_TAG_LENGTH];
    memset(key1, 0, sizeof(key1));
    memset(key2, 0, sizeof(key2));
    int value;
    for (i = 1; i <= nsize; i++)
    {
        value = rand() % 4000 + 1;
        key1[i] = value;
        value = rand() % 4000 + 1;
        key2[i] = value;
    }
    for (i = 1; i < nsize; i++)
    {
        for (j = i+1; j <= nsize; j++)
        {
            if (key1[i] > key1[j])
            {
                tmp = key1[j];
                key1[j] = key1[i];
                key1[i] = tmp;
            }
            if (key2[i] > key2[j])
            {
                tmp = key2[j];
                key2[j] = key2[i];
                key2[i] = tmp;
            }
        }
    }
    int encode1[MAX_TASK_TAG_LENGTH];
    memset(encode1, 0, sizeof(encode1));
    for (i = 1; i <= nsize; i++)
    {
        encode1[parent1[i]] = key1[i];
    }
    int encode2[MAX_TASK_TAG_LENGTH];
    memset(encode2, 0, sizeof(encode2));
    for (i = 1; i <= nsize; i++)
    {
        encode2[parent2[i]] = key2[i];
    }
    int idx1, idx2;
    rand_selection(&idx1, &idx2, nsize);
    idx1 = 4;
    idx2 = 5;
    if (idx1 > idx2)
    {
        tmp = idx2;
        idx2 = idx1;
        idx1 = tmp;
    }
    for (i = idx1; i <= idx2; i++)
    {
        tmp = encode2[i];
        encode2[i] = encode1[i];
        encode1[i] = tmp;
    }
    child[0] = nsize;
    for (i = 1; i <= nsize; i++)
    {
        child[i] = i;
    }
    for (i = 1; i <= nsize; i++)
    {
        for (j = i+1; j <= nsize; j++)
        {
            if (encode1[i] > encode1[j])
            {
                tmp = encode1[i];
                encode1[i] = encode1[j];
                encode1[j] = tmp;

                tmp = child[i];
                child[i] = child[j];
                child[j] = tmp;
            }
        }
    }
}

int MASDC_split(int *split_task_seq, int *one_task_seq, const int *stop, const int *remain_capacity, const Task *inst_tasks)
{
    int one_task_seq_tmp[MAX_TASK_SEQ_LENGTH];
    memcpy(one_task_seq_tmp, one_task_seq, sizeof(one_task_seq_tmp));

    int min_total_cost = INF;
    for (int t=1; t <= 3; t++)
    {
        int i, j, k;
        int position[MAX_TASK_TAG_LENGTH];
        position[0] = one_task_seq[0]-1;
        for (i = 1; i < one_task_seq[0]; i++)
        {
            position[i] = i;
        }
        int split_position[101];
        memset(split_position, 0, sizeof(split_position));
        split_position[0] = 1;
        split_position[1] = 0;
        for (i = 1; i < stop[0]; i++)
        {
            k = rand_choose(position[0]);
            split_position[0] ++;
            split_position[split_position[0]] = position[k];
            delete_element(position, k);
        }
        split_position[0] ++;
        split_position[split_position[0]] = one_task_seq[0];

        for (i = 1; i < stop[0]; i++)
        {
            for (j = i+1; j <= stop[0]; j++)
            {
                if(split_position[i] > split_position[j])
                {
                    int tmp = split_position[j];
                    split_position[j] = split_position[i];
                    split_position[i] = tmp;
                }
            }
        }

        int Route[101][MAX_TASK_SEQ_LENGTH];
        memset(Route, 0, sizeof(Route));
        Route[0][0] = stop[0];
        for (i = 1; i < split_position[0]; i++)
        {
            AssignSubArray(one_task_seq, split_position[i]+1, split_position[i+1], Route[i]);
        }
//        for (i = 1; i <= one_task_seq_tmp[0]; i++)
//        {
//            printf("%d ", one_task_seq_tmp[i]);
//        }
//        printf("\n");
//        for (i = 1; i <= Route[0][0]; i++)
//        {
//            for (j = 1; j <= Route[i][0]; j++)
//            {
//                printf("%d ", Route[i][j]);
//            }
//        }
//        printf("\n");

        int min_dp, dp;
        int assign[101], used[101];
        memset(used, 0, sizeof(used));
        memset(assign, 0, sizeof(assign));
        for (i = 1; i <= Route[0][0]; i++)
        {
            min_dp = INF;
            for (j = 1; j <= stop[0]; j++)
            {
                if (used[j])
                    continue;
                dp = 0;
                for (k = 1; k <= Route[i][0]; k++)
                {
                    dp += min_cost[stop[j]][inst_tasks[Route[i][k]].head_node];
                    dp += min_cost[stop[j]][inst_tasks[Route[i][k]].tail_node];
                }
                if (dp < min_dp)
                {
                    assign[i] = j;
                }
            }
            used[assign[i]] = i;
        }

        // execute
        int prev, load, total_cost = 0;
        for(i = 1; i <= Route[0][0]; i++)
        {
            prev = stop[assign[i]];
            load = capacity - remain_capacity[assign[i]];
            for (j = 1; j <= Route[i][0]; j++)
            {
                if (load + inst_tasks[Route[i][j]].demand > capacity)
                {
                    load = 0;
                    total_cost += min_cost[prev][DEPOT];
                    prev = DEPOT;
                }
                load += inst_tasks[Route[i][j]].demand;
                total_cost += min_cost[prev][inst_tasks[Route[i][j]].head_node];
                total_cost += inst_tasks[Route[i][j]].serv_cost;
                prev = inst_tasks[Route[i][j]].tail_node;
            }
            total_cost += min_cost[prev][DEPOT];
        }

        if (total_cost < min_total_cost)
        {
            memset(split_task_seq, 0, MAX_TASK_SEQ_LENGTH*sizeof(int));
            split_task_seq[0] = 1;
            split_task_seq[1] = 0;
            for (i = 1; i <= stop[0]; i++)
            {
                for (j = 1; j <= Route[used[i]][0]; j++)
                {
                    split_task_seq[0]++;
                    split_task_seq[split_task_seq[0]] = Route[used[i]][j];
                }
                split_task_seq[0]++;
                split_task_seq[split_task_seq[0]] = 0;
            }
            min_total_cost = total_cost;
        }
        // int split_task_seq_tmp[MAX_TASK_SEQ_LENGTH];
        // memcpy(split_task_seq_tmp, split_task_seq, sizeof(split_task_seq_tmp));
//        printf("total cost: %d \n", total_cost);
    }
    return min_total_cost;
}


void cost_test(const int *split, const int *stop, const Task *indi_tasks)
{
    int i;
    
}


























