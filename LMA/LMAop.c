//
// Created by hao on 19/06/2020.
//

# include "LMA.h"
void display_indi(Individual *indi);

// int* rand_perm(int num)
// {
//     int *a = (int *)malloc((num+1)*sizeof(int));
//     int *left_ele = (int *)malloc((num+1)*sizeof(int));
//     left_ele[0] = num;
//     for (int i = 1; i <= num; i++)
//     {
//         left_ele[i] = i;
//     }

//     a[0] = num;
//     for (int i = 1; i <= num; i++)
//     {
//         int k = rand_choose(left_ele[0]);
//         a[i] = left_ele[k];
//         delete_element(left_ele, k);
//     }

//     free(left_ele);

//     return a;
// }


void evaluate_route(Individual *indi, const Task *inst_tasks)
{
    indi->Assignment[0] = 0;
    for (int i = 1; i <= indi->Sequence[0]; i++)
    {
        if (indi->Sequence[i] != 0)
        {
            indi->Assignment[0] ++;
            indi->Assignment[indi->Assignment[0]] = indi->Sequence[i];
        }
    }

    indi->TotalCost = split(indi->Sequence, indi->Assignment, inst_tasks);
}

void OX(Individual *xed_child, Individual *p1, Individual *p2, const Task *inst_tasks)
{
    int i, j, k;
    int tau = req_edge_num + req_arc_num;
    int LeftTasks[2*req_edge_num + req_arc_num + 1];
    memset(LeftTasks, 1, sizeof(LeftTasks));

    int child[tau + 1];
    memset(child, 0, sizeof(child));
    child[0] = tau;

    int p, q;
    p = rand_choose(tau);
    if (p == 1)
    {
        q = rand_choose(tau-1);
    } else {
        q = rand_choose(tau-p+1) + p - 1;
    }
    for (i = p; i <= q; i++)
    {
        child[i] = p1->Assignment[i];
        LeftTasks[p1->Assignment[i]] = 0;
        LeftTasks[inst_tasks[p1->Assignment[i]].inverse] = 0;
    }
    i = q % tau + 1;
    j = i;

    int istart = i;

    while (1)
    {
        if (LeftTasks[p2->Assignment[i]])
        {
            child[j] = p2->Assignment[i];
            LeftTasks[p2->Assignment[i]] = 0;
            LeftTasks[inst_tasks[p2->Assignment[i]].inverse] = 0;
            j = j % tau + 1;
        }
        i = i % tau + 1;
        if (i == istart)
            break;
    }

    memcpy(xed_child->Assignment, child, sizeof(child));
}

void sort_pop(Individual *pop, int popsize)
// sort population in increasing cost order.
{
    Individual tmp_indi;
    for (int i = 0; i < popsize-1; i++)
    {
        for (int j = i + 1; j < popsize; j++)
        {
            if (pop[i].TotalCost > pop[j].TotalCost)
            {
                tmp_indi = pop[i];
                pop[i] = pop[j];
                pop[j] = tmp_indi;
            }
        }
    }

}

void partial_replacement(Individual *pop, int nc, int nrep, const Task *inst_tasks)
{
    int i, j, k, done = 0;
    Individual tmp_indi, best_indi;
    while (done != nrep)
    {
        Individual pool[nrep];
        int pool_size = 0;
        int used;
        while (pool_size < nrep)
        {
            int trial = 0;
            Individual init_indi;

            while (trial < 50)
            {
                trial ++;
                int chromosome[MAX_TASK_SEQ_LENGTH];
                memset(chromosome, 0, sizeof(chromosome));
                rand_perm(chromosome, req_edge_num+req_arc_num);
                memcpy(init_indi.Assignment, chromosome, sizeof(int)*(chromosome[0]+1));
                init_indi.TotalCost = split(init_indi.Sequence, init_indi.Assignment, inst_tasks);
                used = 0;
                for (i = 0; i < nc; i++)
                {
                    if (init_indi.TotalCost == pop[i].TotalCost)
                    {
                        used = 1;
                        break;
                    }
                }
                if ( !used )
                    break;
            }
            if (trial == 50 && used == 1)
                break;

            pool[pool_size] = init_indi;
            pool_size ++;
        }
        sort_pop(pool, pool_size);
        k = 0;


        while (done != nrep && k != nrep)
        {
            best_indi.TotalCost = INF;
            k ++;
            if (pool[k-1].TotalCost < pop[nc-1].TotalCost)
            {
//                display_indi(&pool[k-1]);
//                printf("case1: (%d, %d)\n", pool[k-1].TotalCost, get_task_seq_total_cost(best_indi.Sequence, inst_tasks));
                pop[nc-1] = pool[k-1];
                done ++;
                sort_pop(pop, nc);
            }
            else
            {
                for (i = 0; i < nc; i++)
                {
                    OX(&tmp_indi, &pool[k-1], &pop[i], inst_tasks);
                    tmp_indi.TotalCost = split(tmp_indi.Sequence, tmp_indi.Assignment, inst_tasks);
                    if (tmp_indi.TotalCost < best_indi.TotalCost)
                    {
                        used = 0;
                        for (j = 0; j < nc; j++)
                        {
                            if (tmp_indi.TotalCost == pop[j].TotalCost)
                            {
                                used = 1;
                                break;
                            }
                        }
                        if (!used)
                            best_indi = tmp_indi;
                    }
                }

                for (i = 0; i < pool_size; i++)
                {
                    if (i == k-1)
                        continue;

                    OX(&tmp_indi, &pool[k-1], &pool[i], inst_tasks);
                    tmp_indi.TotalCost = split(tmp_indi.Sequence, tmp_indi.Assignment, inst_tasks);
                    if (tmp_indi.TotalCost < best_indi.TotalCost)
                    {
                        used = 0;
                        for (j = 0; j < nc; j++)
                        {
                            if (tmp_indi.TotalCost == pop[j].TotalCost)
                            {
                                used = 1;
                                break;
                            }
                        }
                        if (!used)
                            best_indi = tmp_indi;
                    }
                }

                if (best_indi.TotalCost < pop[nc-1].TotalCost)
                {
//                    display_indi(&best_indi);
//                    printf("case2: (%d, %d)\n", best_indi.TotalCost, get_task_seq_total_cost(best_indi.Sequence, inst_tasks));
                    pop[nc-1] = best_indi;
                    done ++;
                    sort_pop(pop, nc);
                }
            }
        }
    }

}

void display_indi(Individual *indi)
{
    int i;
    for (i = 1; i <= indi->Assignment[0]; i++)
    {
        printf("%d  ", indi->Assignment[i]);
    }
    printf("\n");
}














