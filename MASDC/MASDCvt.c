//
// Created by hao on 19/09/2020.
//

#include "MASDC.h"

#define MAX_POPSIZE 30
#define M_trial 10
int MASDCvt_local_search(Individual *indi, Individual *child, const Task *inst_tasks, const int max_fe);

void check_seq(Individual *solution2);
void check_seq(Individual *solution2)
{
    int temp_task[300], i, j;
    memset(temp_task, 0, sizeof(temp_task));
    for (i=1; i <= solution2->Sequence[0]; i++)
    {
        int task = solution2->Sequence[i];
        if (task == 0)
            continue;
        
        if (task > req_edge_num)
            task -= req_edge_num;
        
        if (temp_task[task] == 0)
        {
            temp_task[task] = 1;
        } else {
            printf("%d ", task);
        }
        
    }
    int flag = 0;
    for (i = 1; i <= req_edge_num; i++)
    {
        if (temp_task[i] == 0)
        {
            printf("%d ", i);
            flag = 1;
        }
    }
    if (flag)
    {
        printf("\n%d %d \n -------->", solution2->Assignment[0], req_edge_num);
    }
}


void MASDCvt(Individual *best_individual, Task *inst_tasks)
{

    int fe = 0;
    int max_fe = vertex_num * 200;

    int i, j, k;
    int popsize = 30;
    Individual pop[MAX_POPSIZE+1];

    int tmp_popsize = 1;
    int used;
    int bestf = INF;
    while (tmp_popsize <= popsize)
    {
        int trial = 0;
        Individual init_indi;
        while (trial < M_trial)
        {
            trial ++;
            int serve_mark[MAX_TASK_TAG_LENGTH];
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

        init_indi.TotalCost = split(init_indi.Sequence, init_indi.Assignment, inst_tasks);
        check_seq(&init_indi);
        fe += 1;
        if (init_indi.TotalCost < bestf)
        {
            bestf = init_indi.TotalCost;
        }
        pop[tmp_popsize] = init_indi;
        tmp_popsize ++;
    }
    popsize = tmp_popsize-1;

    int choose[3];
    memset(choose, 0, sizeof(choose));
    Individual parent1, parent2, xed_child, mted_child, child;

    clock_t start_t, finish_t;
    start_t = clock();


    // printf("fe: %d, best: %d \n", fe, bestf);
    // main loop
    
    int best = 0, worst = 0;
    double duration, duration1=0;

    int stop_iter = 0, old_best = INF;
    int iter = 0, max_iter = 900 * (int)sqrt(req_edge_num + nonreq_edge_num);

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
        xed_child.TotalCost = split(xed_child.Sequence, xed_child.Assignment, inst_tasks);
        fe += 1;
        child = xed_child;

        // apply local search
        double random = 1.0 * rand() / RAND_MAX;
        if (random < 1)
        {
            memset(mted_child.Assignment, 0, sizeof(mted_child.Assignment));
            int tmp_fe;
            tmp_fe = MASDCvt_local_search(&xed_child, &mted_child, inst_tasks, max_fe-fe);
            fe += tmp_fe;
            

            if (mted_child.TotalCost == INF)
            {
                mted_child = child;
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

        for (i = 1; i <= popsize; i++)
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
        // fe += 100; // required to be update
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
    printf("MASDCvt: Final: fe: %d, best: %d \n", fe, best_individual->TotalCost);

}



void MASDCvtih(Individual *best_individual, Task *inst_tasks, Individual InitSolution)
{

    int fe = 0;
    int max_fe = vertex_num * 200;

    int i, j, k;
    int popsize = 30;
    Individual pop[MAX_POPSIZE+1];

    pop[0] = InitSolution;

    int tmp_popsize = 1;
    int used;
    int bestf = INF;
    while (tmp_popsize <= popsize)
    {
        int trial = 0;
        Individual init_indi;
        while (trial < M_trial)
        {
            trial ++;
            int serve_mark[MAX_TASK_TAG_LENGTH];
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

        init_indi.TotalCost = split(init_indi.Sequence, init_indi.Assignment, inst_tasks);
        check_seq(&init_indi);
        fe += 1;
        if (init_indi.TotalCost < bestf)
        {
            bestf = init_indi.TotalCost;
        }
        pop[tmp_popsize] = init_indi;
        tmp_popsize ++;
    }
    popsize = tmp_popsize-1;

    int choose[3];
    memset(choose, 0, sizeof(choose));
    Individual parent1, parent2, xed_child, mted_child, child;

    // printf("fe: %d, best: %d \n", fe, bestf);
    // main loop
    

    clock_t start_t, finish_t;
    start_t = clock();


    int best = 0, worst = 0;

    double duration, duration1=0;

    int stop_iter = 0, old_best = INF;
    int iter = 0, max_iter = 900 * (int)sqrt(req_edge_num + nonreq_edge_num);
    // while (iter < max_iter) fe < max_fe iter
    while (duration < 2*terminal_duration && stop_iter <= terminal_condition)
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
        xed_child.TotalCost = split(xed_child.Sequence, xed_child.Assignment, inst_tasks);
        fe += 1;
        child = xed_child;

        // apply local search
        double random = 1.0 * rand() / RAND_MAX;
        if (random < 1)
        {
            memset(mted_child.Assignment, 0, sizeof(mted_child.Assignment));
            int tmp_fe;
            tmp_fe = MASDCvt_local_search(&xed_child, &mted_child, inst_tasks, max_fe-fe);
            fe += tmp_fe;
            if (mted_child.TotalCost == INF)
            {
                mted_child = child;
            }
            // check_seq(&mted_child);
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

        for (i = 1; i <= popsize; i++)
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
            if(best_individual->TotalCost != get_task_seq_total_cost(best_individual->Sequence, inst_tasks))
            {
                printf("MASDCVTih: error, ");
                exit(0);
                // longjmp(buf, 2);
            }
        }
        // fe += 100; // required to be update
        iter ++;
        // printf("fe: %d, best: %d \n", fe, best_individual->TotalCost);
        if (old_best > best_individual->TotalCost)
        {
            stop_iter = 0;
            old_best = best_individual->TotalCost;
        } else
        {
            stop_iter ++;
        }
        finish_t = clock();
        // fp = fopen(path_trend, "a");
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        // fclose(fp);
    }
    printf("MASDCvtih: Final: fe: %d, best: %d \n", fe, best_individual->TotalCost);

}





void check_lcoal_seq(int *seq);
void check_lcoal_seq(int *seq)
{
    int used[300], i, j;
    memset(used, 0, sizeof(used));
    for (i = 1; i <= seq[0]; i++)
    {
        int task = seq[i];
        if (task == 0)
            continue;
        
        if (task > req_edge_num)
            task -= req_edge_num;
        
        if (used[task] == 0)
        {
            used[task] = 1;
        } else {
            printf("%d ", task);
        }
    }
    int flag = 0;
    for (i = 1; i <= req_edge_num; i++)
    {
        if (used[i] == 0)
        {
            printf("%d ", i);
            flag = 1;
        }
    }
    if (flag)
    {
        printf("\n%d %d +++++++++ \n", seq[0], req_edge_num);
    }
}

int MASDCvt_local_search(Individual *indi, Individual *child, const Task *inst_tasks, const int max_fe)
{
    int i, j, k;
    int fe = 0;
    int flag = 0;

    int new_seq[MAX_TASK_SEQ_LENGTH], best_seq[MAX_TASK_SEQ_LENGTH];
    int fitness, best_fitness = INF;
    int best_split[MAX_TASK_SEQ_LENGTH], new_split[MAX_TASK_SEQ_LENGTH];
    int task1, task2;
    // single Insertion;
    for (i = 1; i < indi->Assignment[0]; i++) {
        task1 = indi->Assignment[i];
        memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
        delete_element(new_seq, i);
        for (j = i + 1; j <= indi->Assignment[0]; j++) {
            add_element(new_seq, task1, j);
            check_lcoal_seq(new_seq);

            fitness = split(new_split, new_seq, inst_tasks);
            fe += 1;
            // if (fe > max_fe)
            // {
            //     return fe;
            // }
            if (fitness < best_fitness) {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
                flag = 1;
            }
            delete_element(new_seq, j);
        }
    }
    // double insertion
    for (i = 1; i < indi->Assignment[0] - 1; i++) {
        task1 = indi->Assignment[i];
        task2 = indi->Assignment[i + 1];
        memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
        for (k = i; k < new_seq[0] - 1; k++) {
            new_seq[k] = new_seq[k + 2];
        }

        for (j = 1; j <= indi->Assignment[0] - 2; j++) {
            // can be optimized here
            for (k = new_seq[0]; k > j + 1; k--) {
                new_seq[k] = new_seq[k - 2];
            }
            new_seq[j] = task1;
            new_seq[j + 1] = task2;

            check_lcoal_seq(new_seq);
            fitness = split(new_split, new_seq, inst_tasks);
            fe += 1;
            // if (fe > max_fe)
            // {
            //     return fe;
            // }
            if (fitness < best_fitness) {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
                flag = 1;
            }

            for (k = j; k < new_seq[0] - 1; k++) {
                new_seq[k] = new_seq[k + 2];
            }

        }
    }

    // swap
    memcpy(new_seq, indi->Assignment, sizeof(indi->Assignment));
    for (i = 1; i < indi->Assignment[0]; i++) {
        for (j = i + 1; j <= indi->Assignment[0]; j++) {
            int tmp = new_seq[i];
            new_seq[i] = new_seq[j];
            new_seq[j] = tmp;
            check_lcoal_seq(new_seq);
            fitness = split(new_split, new_seq, inst_tasks);
            fe += 1;
            // if (fe > max_fe)
            // {
            //     return fe;
            // }
            if (fitness < best_fitness) {
                // printf("best: %d\n", fitness);
                best_fitness = fitness;
                memcpy(best_split, new_split, sizeof(new_split));
                memcpy(best_seq, new_seq, sizeof(new_seq));
                flag = 1;
            }
            tmp = new_seq[i];
            new_seq[i] = new_seq[j];
            new_seq[j] = tmp;
        }
    }


    
    if (flag == 0)
    {
        printf("No imporoved \n");
    }
    memcpy(child->Sequence, best_split, sizeof(best_split));
    memcpy(child->Assignment, best_seq, sizeof(best_seq));
    child->TotalCost = best_fitness;
    check_seq(child);
    
    return fe;
}