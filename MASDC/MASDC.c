//
// Created by hao on 27/07/2020.
//

#include "MASDC.h"

#define MAX_POPSIZE 30
#define M_trial 10



void MASDC(Individual *best_individual, Task *inst_tasks, const int *stop, const int *remain_capacity)
{
    int fe = 0, max_fe = vertex_num * 200;
    int i, j, k;
    int popsize = 30;
    Individual pop[MAX_POPSIZE+1];

    int tmp_popsize = 1;
    int used;
    int bestf = INF;
    while (tmp_popsize <= popsize)
    {
        int trial = 0;
        int serve_mark[MAX_TASK_TAG_LENGTH];
        Individual init_indi;
        while (trial < M_trial)
        {
            trial ++;
            memset(serve_mark, 0, sizeof(serve_mark));
            for (i = 1; i <= task_num; i++)
            {
                serve_mark[i] = 1;
            }
            rand_seq(&init_indi, inst_tasks, serve_mark);
            used = 0;
            for (i = 0; i < tmp_popsize; i++)
            {
                used = same_or_not(init_indi.Assignment, pop[i].Assignment);
                if (used)
                    break;
            }
            if ( !used )
                break;
        }
        if (trial == M_trial && used == 1)
            break;

        init_indi.TotalCost = MASDC_split(init_indi.Sequence, init_indi.Assignment, stop, remain_capacity, inst_tasks);
        // check_solution_valid(init_indi, inst_tasks);
        if (init_indi.TotalCost < bestf)
        {
            bestf = init_indi.TotalCost;
        }
        fe += 3;
        pop[tmp_popsize] = init_indi;
        if (init_indi.Sequence[0] == 1)
        {
            printf("error \n");
        }
        tmp_popsize ++;
    }
    popsize = tmp_popsize-1;


    clock_t start_t, finish_t;
    start_t = clock();



    int choose[3];
    memset(choose, 0, sizeof(choose));
    Individual parent1, parent2, xed_child, mted_child, child;

    // printf("fe: %d, best: %d \n", fe, bestf);lo  l
    // main loop
    int best = 0, worst = 0;

    double duration, duration1=0.0;

    int stop_iter = 0, old_best = INF;
    int iter = 0, max_iter = 900 * (int)sqrt(req_edge_num + nonreq_edge_num); //

    while (duration < terminal_duration && stop_iter <= terminal_condition)
    {

        // apply Route-Wheel selection to select two parents
        int fitness[31];
        for (i = 1; i <= popsize; i++)
        {
            fitness[i] = pop[i].TotalCost;
        }
        fitness[0] = popsize;

        roulette_wheel_selection(fitness, choose);

        parent1 = pop[choose[1]];
        parent2 = pop[choose[2]];
        // apply crossover
        memset(xed_child.Assignment, 0, sizeof(xed_child.Assignment));
        MASDC_crossover(parent1.Assignment, parent2.Assignment, xed_child.Assignment);
        xed_child.TotalCost = MASDC_split(xed_child.Sequence, xed_child.Assignment, stop, remain_capacity, inst_tasks);


        fe += 3;
        child = xed_child;

        // apply local search
        double random = 1.0 * rand() / RAND_MAX;
        if (random < 0.2)
        {
            memset(mted_child.Assignment, 0, sizeof(mted_child.Assignment));
            int tmp_fe;
            tmp_fe = MASDC_local_search(&xed_child, &mted_child, stop, remain_capacity, inst_tasks, max_fe - fe);

            fe += tmp_fe;
 
            if (mted_child.TotalCost == INF)
            {
                mted_child = child;
            }
            if (mted_child.Sequence[0] == 0)
            {
                printf("error \n");
            }
            for (i = 1; i <= popsize; i++)
            {
                used = same_or_not(mted_child.Assignment, pop[i].Assignment);
                if (used)
                    break;
            }
            if (!used)
            {
                child = mted_child;
            }
        }

        for (i = 0; i < popsize; i++)
        {
            used = same_or_not(child.Assignment, pop[i].Assignment);
            if (used)
                break;
        }
        int best_fitness = INF, worst_fitness = 0;
        if (!used)
        {
            // add child to population and sort population and select the new population
            for (i = 1; i <= popsize; i++)
            {
                if (pop[i].TotalCost > worst_fitness)
                {
                    worst = i;
                    worst_fitness = pop[i].TotalCost;
                }
                if (pop[i].TotalCost < best_fitness)
                {
                    best = i;
                    best_fitness = pop[i].TotalCost;
                }
            }
            pop[worst] = child;
            if (child.TotalCost < pop[best].TotalCost)
            {
                best = worst;
                best_fitness = child.TotalCost;
            }
            memcpy(best_individual->Sequence, pop[best].Sequence, sizeof(pop[best].Sequence));
            memcpy(best_individual->Assignment, pop[best].Assignment, sizeof(pop[best].Assignment));
            best_individual->TotalCost = pop[best].TotalCost;
        }
        iter ++;

        if (old_best > best_individual->TotalCost)
        {
            stop_iter = 0;
            old_best = best_individual->TotalCost;
        } else
        {
            stop_iter ++;
        }
        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;

    }
    printf("MASDC: Final: fe: %d, best: %d \n", fe, best_individual->TotalCost);
    


}

int same_or_not(const int *seq1, const int *seq2)
{
    int i;
    int k = seq1[0];
    for (i = 1; i <= k; i++)
    {
        if (seq1[i] != seq2[i])
        {
            return 0;
        }
    }
    return 1;
}

void rand_seq(Individual *rs_indi, const Task *inst_tasks, const int *serve_mark)
{
    int i, k;
    int serve_task_num = 0; // the number of tasks required to be served.
    for (i = req_edge_num+1; i <= task_num; i++)
    {
        if(serve_mark[i])
            serve_task_num++;
    }

    int trial;

    int unserved_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH];
    int next_task;

    int positions[MAX_TASK_SEG_LENGTH];


    unserved_task[0] = 0;

    for (i = 1; i <= task_num; i++)
    {
        if ( !serve_mark[i] )
            continue;

        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    trial = 0;
    memset(rs_indi->Assignment, 0, sizeof(rs_indi->Assignment));
    while (trial < serve_task_num)
    {
        candi_task[0] = 0;
        for( i = 1; i <= unserved_task[0]; i++)
        {
            candi_task[0] ++;
            candi_task[candi_task[0]] = unserved_task[i];
        }

        k = rand_choose(candi_task[0]);
        next_task = candi_task[k];

        trial ++;
        rs_indi->Assignment[0] ++;
        rs_indi->Assignment[rs_indi->Assignment[0]] = next_task;
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task, positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

}
