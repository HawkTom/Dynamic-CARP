//
// Created by hao on 15/06/2020.
//

#include "MAENS.h"

void ShowIndi(Individual *pop, int popsize);

void indi_route_converter2(Individual *dst, Individual *src, const Task *inst_tasks);
void MAENS(const Task *inst_tasks, Individual *MAENSolution)
{
    int i, j;

    int popsize = 30;
    Individual pop[MAX_TOTALSIZE];
    Individual best_fsb_solution;

    best_fsb_solution.TotalCost = INF;

    // initilization
    int tmp_popsize = 0;
    int used;
    while (tmp_popsize < popsize)
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

            rand_scanning(&init_indi, inst_tasks, serve_mark);
            used = 0;
            for (i = 0; i < tmp_popsize; i++)
            {
                if (init_indi.TotalCost == pop[i].TotalCost && init_indi.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }
            if ( !used )
                break;
        }

        if (trial == M_trial && used == 1)
            break;


        pop[tmp_popsize] = init_indi;
        tmp_popsize ++;
        if(init_indi.TotalVioLoad == 0 && init_indi.TotalCost < best_fsb_solution.TotalCost)
        {
            best_fsb_solution = init_indi;
        }
    }
    popsize = tmp_popsize;

    // main loop


    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0, duration1=0.0;



    int ite, wite;
    Individual parent1, parent2, xed_child, mted_child, child;

    int offsize = 6*popsize;
    int totalsize = popsize + offsize;

    ite = 0;
    wite = 0;
    int stop_iter = 0, old_best = INF;
    // while (stop_iter < terminal_condition)
    while (ite < M_ite && duration < terminal_duration)
    {
        ite ++;
        wite ++;

        int ptr = popsize;
        while (ptr < totalsize)
        {
            child.TotalCost = 0;

            // randomly select two parents
            int par_id1, par_id2;
            rand_selection(&par_id1, &par_id2, popsize);
            parent1 = pop[par_id1];
            parent2 = pop[par_id2];

            // crossover
            SBX(&xed_child, &parent1, &parent2, inst_tasks);
            if (xed_child.TotalVioLoad == 0 && xed_child.TotalCost < best_fsb_solution.TotalCost)
            {
                best_fsb_solution = xed_child;
                wite = 0;
            }

            used = 0;
            for (i = 0; i < ptr; i++)
            {
                if (i == par_id1 || i == par_id2)
                    continue;

                if (xed_child.TotalCost == pop[i].TotalCost && xed_child.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }

            if (!used)
            {
                child = xed_child;
            }

            // Local Search with Probability
            double random = 1.0 * rand() / RAND_MAX;
            if (random < M_PROB)
            {
                // do the local search.
                lns_mut(&mted_child, &xed_child, &best_fsb_solution, inst_tasks);

                used = 0;
                for (i = 0; i < ptr; i++)
                {
                    if (i == par_id1 || i == par_id2)
                        continue;

                    if (mted_child.TotalCost == pop[i].TotalCost && mted_child.TotalVioLoad == pop[i].TotalVioLoad)
                    {
                        used = 1;
                        break;
                    }
                }

                if (!used)
                {
                    child = mted_child;
                }
            }

            if (child.TotalCost == parent1.TotalCost && child.TotalVioLoad == parent1.TotalVioLoad)
            {
                pop[par_id1] = child;
            } else if (child.TotalCost == parent2.TotalCost && child.TotalVioLoad == parent2.TotalVioLoad)
            {
                pop[par_id2] = child;
            } else if (child.TotalCost > 0)
            {
                pop[ptr] = child;
                ptr ++;
            }
            finish_t = clock();
            duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        }

        // stochastic ranking
        float Pf = 0.45;
        Individual tmp_indi;


        for (i = 0; i < totalsize; i++)
        {
            for (j = 0; j < i; j++)
            {
                double random = 1.0 * rand() / RAND_MAX;
                if ( (pop[j].TotalVioLoad == 0 && pop[j+1].TotalVioLoad == 0) || random < Pf )
                {
                    if (pop[j].TotalCost > pop[j+1].TotalCost)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                } else {
                    if (pop[j].TotalVioLoad > pop[j+1].TotalVioLoad)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                }
            }
        }
        if (best_fsb_solution.TotalCost < old_best)
        {
            stop_iter = 0;
            old_best = best_fsb_solution.TotalCost;
        } else
        {
            stop_iter ++;
        }

        
    }

    // printf("MAENS: %d, %d, %d\n", ite, best_fsb_solution.TotalCost, get_task_seq_total_cost(best_fsb_solution.Sequence, inst_tasks));

    // memcpy(MAENSolution->Sequence, best_fsb_solution.Sequence, sizeof(best_fsb_solution.Sequence));
    // memcpy(MAENSolution->Loads, best_fsb_solution.Loads, sizeof(best_fsb_solution.Loads));

    indi_route_converter2(MAENSolution, &best_fsb_solution, inst_tasks);
    memcpy(MAENSolution->Assignment, best_fsb_solution.Assignment, sizeof(best_fsb_solution.Assignment));
    MAENSolution->TotalCost = best_fsb_solution.TotalCost;
    MAENSolution->TotalVioLoad = best_fsb_solution.TotalVioLoad;
    MAENSolution->Fitness = best_fsb_solution.Fitness;

}


void MAENSih(const Task *inst_tasks, Individual *MAENSolution, Individual InitSolution)
{
    int i, j;

    int popsize = 30;
    Individual pop[MAX_TOTALSIZE];
    Individual best_fsb_solution;

    best_fsb_solution.TotalCost = INF;

    pop[0] = InitSolution;

    // initilization
    int tmp_popsize = 1;
    int used;
    while (tmp_popsize < popsize)
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

            rand_scanning(&init_indi, inst_tasks, serve_mark);
            used = 0;
            for (i = 0; i < tmp_popsize; i++)
            {
                if (init_indi.TotalCost == pop[i].TotalCost && init_indi.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }
            if ( !used )
                break;
        }

        if (trial == M_trial && used == 1)
            break;


        pop[tmp_popsize] = init_indi;
        tmp_popsize ++;
        if(init_indi.TotalVioLoad == 0 && init_indi.TotalCost < best_fsb_solution.TotalCost)
        {
            best_fsb_solution = init_indi;
        }
    }
    popsize = tmp_popsize;

    // main loop

    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0, duration1=0.0;


    int ite, wite;
    Individual parent1, parent2, xed_child, mted_child, child;

    int offsize = 6*popsize;
    int totalsize = popsize + offsize;

    ite = 0;
    wite = 0;
    int stop_iter = 0, old_best = INF;
    // while (stop_iter < terminal_condition)
    while (ite < M_ite && duration < terminal_duration)
    {
        ite ++;
        wite ++;

        int ptr = popsize;
        while (ptr < totalsize)
        {
            child.TotalCost = 0;

            // randomly select two parents
            int par_id1, par_id2;
            rand_selection(&par_id1, &par_id2, popsize);
            parent1 = pop[par_id1];
            parent2 = pop[par_id2];

            // crossover
            SBX(&xed_child, &parent1, &parent2, inst_tasks);
            if (xed_child.TotalVioLoad == 0 && xed_child.TotalCost < best_fsb_solution.TotalCost)
            {
                best_fsb_solution = xed_child;
                wite = 0;
            }

            used = 0;
            for (i = 0; i < ptr; i++)
            {
                if (i == par_id1 || i == par_id2)
                    continue;

                if (xed_child.TotalCost == pop[i].TotalCost && xed_child.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }

            if (!used)
            {
                child = xed_child;
            }

            // Local Search with Probability
            double random = 1.0 * rand() / RAND_MAX;
            if (random < M_PROB)
            {
                // do the local search.
//                printf("Local search %d \n", inst_tasks[1].inverse);
                lns_mut(&mted_child, &xed_child, &best_fsb_solution, inst_tasks);

                used = 0;
                for (i = 0; i < ptr; i++)
                {
                    if (i == par_id1 || i == par_id2)
                        continue;

                    if (mted_child.TotalCost == pop[i].TotalCost && mted_child.TotalVioLoad == pop[i].TotalVioLoad)
                    {
                        used = 1;
                        break;
                    }
                }

                if (!used)
                {
                    child = mted_child;
                }
            }

            if (child.TotalCost == parent1.TotalCost && child.TotalVioLoad == parent1.TotalVioLoad)
            {
                pop[par_id1] = child;
            } else if (child.TotalCost == parent2.TotalCost && child.TotalVioLoad == parent2.TotalVioLoad)
            {
                pop[par_id2] = child;
            } else if (child.TotalCost > 0)
            {
                pop[ptr] = child;
                ptr ++;
            }
            finish_t = clock();
            duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        }
//        if (best_fsb_solution.TotalCost == lower_bound)
//                break;

        // stochastic ranking
        float Pf = 0.45;
        Individual tmp_indi;


        for (i = 0; i < totalsize; i++)
        {
            for (j = 0; j < i; j++)
            {
                double random = 1.0 * rand() / RAND_MAX;
                if ( (pop[j].TotalVioLoad == 0 && pop[j+1].TotalVioLoad == 0) || random < Pf )
                {
                    if (pop[j].TotalCost > pop[j+1].TotalCost)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                } else {
                    if (pop[j].TotalVioLoad > pop[j+1].TotalVioLoad)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                }
            }
        }
        if (best_fsb_solution.TotalCost < old_best)
        {
            stop_iter = 0;
            old_best = best_fsb_solution.TotalCost;
        } else
        {
            stop_iter ++;
        }
    }
    printf("MAENSih: %d: %d \n", ite, best_fsb_solution.TotalCost);

    // memcpy(MAENSolution->Sequence, best_fsb_solution.Sequence, sizeof(best_fsb_solution.Sequence));
    // memcpy(MAENSolution->Loads, best_fsb_solution.Loads, sizeof(best_fsb_solution.Loads));

    indi_route_converter2(MAENSolution, &best_fsb_solution, inst_tasks);
    memcpy(MAENSolution->Assignment, best_fsb_solution.Assignment, sizeof(best_fsb_solution.Assignment));
    MAENSolution->TotalCost = best_fsb_solution.TotalCost;
    MAENSolution->TotalVioLoad = best_fsb_solution.TotalVioLoad;
    MAENSolution->Fitness = best_fsb_solution.Fitness;

}

void indi_route_converter2(Individual *dst, Individual *src, const Task *inst_tasks)
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


void ShowIndi(Individual *pop, int popsize)
{
    for (int i= 0; i < popsize; i++)
    {
        printf("%d, TC: %d, TVIO: %d \n", i, pop[i].TotalCost, pop[i].TotalVioLoad);
    }
}
















