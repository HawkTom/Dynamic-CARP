//
// Created by hao on 13/06/2020.
//

# include "TSA.h"

int SingleInsertion(Individual *BestSINeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
        double PenPrmt, double BestFitness, double BestFsbFitness )
{
    int i, j, k, u, v, z;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    Individual Neighbor;

    MovedTasks[0] = 1;
    BestSINeighbor->Fitness = INF;
    Bestu = -1;

    for (i = 1; i < Positions[0]; i++)
    {
        RID1 = i;
        for (j = 2; j < Route[i][0]; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = 0; u < Positions[0]; u++)
            {
                if (u == i)
                    continue; // insertion happens in two different routes

                RID2 = u;
                if (u == 0 && Route[0][0]<vehicle_num && Route[i][0] > 3)
                {
                    // assign a new route, the route number < max vehicle, over one tasks in Route[i];
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand;
                    } else if ( Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad -= Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - capacity;
                    }

                    Neighbor.Loads[0] ++;
                    Neighbor.Loads[Neighbor.Loads[0]] = inst_tasks[Route[i][j]].demand;

                    if (Neighbor.Loads[Neighbor.Loads[0]] > capacity)
                    {
                        Neighbor.TotalVioLoad += Neighbor.Loads[Neighbor.Loads[0]] - capacity;
                    }
                    int tmp_vio_load = get_total_vio_load(Neighbor.Loads);

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                                - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                                + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                                + min_cost[DEPOT][inst_tasks[Route[i][j]].head_node]
                                                + min_cost[inst_tasks[Route[i][j]].tail_node][DEPOT];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    // check tabu list
                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = 0;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
                if (u == 0)
                    continue;

                for (v = 2; v <= Route[u][0]; v++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
                {
                    if ( u == i && v == j)
                        continue;

                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand;
                    Neighbor.Loads[u] += inst_tasks[Route[i][j]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand;
                    } else if (Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - capacity);
                    }

                    if (Neighbor.Loads[u] > capacity + inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand;

                    } else if (Neighbor.Loads[u] > capacity)
                    {
                        Neighbor.TotalVioLoad += Neighbor.Loads[u] - capacity;
                    }

                    if (Neighbor.Loads[i] == 0)
                    {
                        RID1 = 0;
                        delete_element(Neighbor.Loads, i);
                    }
                    


                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                    // invert selected task
                    z = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if (Bestu < 0)
    {
        return count;
    }

    // Assign Sequence
    if (Bestu == 0)
    {
        // new routes
        BestSINeighbor->Sequence[0] = 1;
        for(k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj);

                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else {
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            }
        }
        BestSINeighbor->Sequence[0] ++;
        BestSINeighbor->Sequence[BestSINeighbor->Sequence[0]] = MovedTasks[1];
        BestSINeighbor->Sequence[0] ++;
        BestSINeighbor->Sequence[BestSINeighbor->Sequence[0]] = 0;

    } else{

        BestSINeighbor->Sequence[0] = 1;
        for(k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj);
                if (Route[k][0] == 2)
                    continue;

                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else if (k == Bestu){
                add_element(Route[k], MovedTasks[1], Bestv);
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else {
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            }
        }
    }

    return count;

}


int DoubleInsertion(Individual *BestDINeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness)
{
    int i, j, k, u, v, z, w;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    Individual Neighbor;

    MovedTasks[0] = 2;
    BestDINeighbor->Fitness = INF;

    Bestu = -1;


    for (i = 1; i < Positions[0]; i++)
    {
        if (Route[i][0] < 4)
            continue;
        RID1 = i;
        for (j = 2; j < Route[i][0]-1; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = 0; u < Positions[0]; u++)
            {
                if (u == i)
                    continue;

                RID2 = u;

                if ( u==0 && Route[0][0] < vehicle_num && Route[i][0] > 4)
                {
                    // new routes
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    } else if ( Neighbor.Loads[i] > (capacity - inst_tasks[Route[i][j]].demand - inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] - capacity + inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    }

                    Neighbor.Loads[0] ++;
                    Neighbor.Loads[Neighbor.Loads[0]] = inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand;

                    if (Neighbor.Loads[Neighbor.Loads[0]] > capacity)
                    {
                        Neighbor.TotalVioLoad += (Neighbor.Loads[Neighbor.Loads[0]] -capacity);
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                                                 - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                                                 + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                                                 + min_cost[DEPOT][inst_tasks[Route[i][j]].head_node]
                                                                 + min_cost[inst_tasks[Route[i][j+1]].tail_node][DEPOT];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                        && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = 0;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                }


                if (u == 0)
                    continue;

                for (v = 2; v <= Route[u][0]; v++)
                {
                    
                    if (u == i && v == j )
                        continue;
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.Loads[u] += (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if ( Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    } else if (Neighbor.Loads[i] > capacity - (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] - capacity + inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    }

                    if (Neighbor.Loads[u] > capacity + (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand;
                    } else if (Neighbor.Loads[u] > capacity)
                    {
                        Neighbor.TotalVioLoad += (Neighbor.Loads[u] - capacity);
                    }
                    if (Neighbor.Loads[i] == 0)
                    {
                        RID1 = 0;
                        delete_element(Neighbor.Loads, i);
                    }
                    
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                       && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    w = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                                                                                             && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    z = inst_tasks[Route[i][j+1]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                        && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                                                                                             && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if (Bestu < 0)
        return count;
    
    if (Bestu == 0)
    {
        BestDINeighbor->Sequence[0] = 1;
        for (k = 1; k < Positions[0]; k++)
        {
            if ( k == Besti )
            {
                delete_element(Route[k], Bestj+1);
                delete_element(Route[k], Bestj);
                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else {
                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            }
        }
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = MovedTasks[1];
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = MovedTasks[2];
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = 0;
    } else{

        BestDINeighbor->Sequence[0] = 1;
        for (k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj+1);
                delete_element(Route[k], Bestj);
                if (Route[k][0] == 2)
                    continue;

                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else if (k == Bestu)
            {
                add_element(Route[k], MovedTasks[2], Bestv);
                add_element(Route[k], MovedTasks[1], Bestv);
                BestDINeighbor->Sequence[0] --;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else{
                BestDINeighbor->Sequence[0] --;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            }
        }
    }

    return count;

}


int SWAP(Individual *BestSWAPNeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
         double PenPrmt, double BestFitness, double BestFsbFitness)
{
    int i, j, k, u, v, z, w;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    Individual Neighbor;

    MovedTasks[0] = 2;
    BestSWAPNeighbor->Fitness = INF;
    Bestu = -1;

    for (i = 1;  i < Route[0][0]; i++)
    {
        RID1 = i;
        for (j = 2; j < Route[i][0]; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = i+1; u <= Route[0][0]; u++)
            {
                RID2 = u;
                for (v = 2; v < Route[u][0]; v++)
                {
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                    Neighbor.Loads[u] += inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (inst_tasks[Route[i][j]].demand > inst_tasks[Route[u][v]].demand)
                    {
                        if (Neighbor.Loads[i] > capacity)
                        {
                            Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                        } else if (Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand + inst_tasks[Route[u][v]].demand)
                        {
                            Neighbor.TotalVioLoad -= Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand - capacity;
                        }

                        if (Neighbor.Loads[u] > capacity + inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand)
                        {
                            Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                        } else if (Neighbor.Loads[u] > capacity)
                        {
                            Neighbor.TotalVioLoad += Neighbor.Loads[u] - capacity;
                        }
                    } else{
                        if (Neighbor.Loads[u] > capacity)
                        {
                            Neighbor.TotalVioLoad -= inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand;
                        }
                        else if (Neighbor.Loads[u] > capacity - inst_tasks[Route[u][v]].demand + inst_tasks[Route[i][j]].demand)
                        {
                            Neighbor.TotalVioLoad -= Neighbor.Loads[u] + inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand - capacity;
                        }

                        if (Neighbor.Loads[i] > capacity + inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand)
                        {
                            Neighbor.TotalVioLoad += inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand;
                        }
                        else if (Neighbor.Loads[i] > capacity)
                        {
                            Neighbor.TotalVioLoad += Neighbor.Loads[i] - capacity;
                        }
                    }
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                            + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                            + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            + min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[u][v];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    w = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = Route[u][v];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    z = inst_tasks[Route[u][v]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if(Bestu < 0)
    {
        return count;
    }
    BestSWAPNeighbor->Sequence[0] = 1;
    for (k = 1; k < Positions[0]; k++)
    {
        if (k == Besti)
        {
            delete_element(Route[k], Bestj);
            add_element(Route[k], MovedTasks[2], Bestj);
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        } else if (k == Bestu)
        {
            delete_element(Route[k], Bestv);
            add_element(Route[k], MovedTasks[1], Bestv);
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        } else{
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        }
    }
    return count;

}

void RepairInfeasibility (Individual *Indi, const Task *inst_tasks)
{
    int NRE = req_edge_num, NRA = req_arc_num;
    int i, j, k, RoutesNum;
    int NodeRoutes[101][MAX_TASK_SEQ_LENGTH], TaskRoutes[101][MAX_TASK_SEQ_LENGTH];
    int InRoutes[NRE+NRA + 1][101], CheckMark[NRE+NRA+1];
    int NO_Task = 2*NRE + NRA;

    // TaskRoutes: each row is the task sequence of each sub-route
    // RoutesNum: the number of routes
    RoutesNum = 0;
    for (i = 1; i < Indi->Sequence[0]; i++)
    {
        if (Indi->Sequence[i] == 0)
        {
            TaskRoutes[RoutesNum][0]++;
            TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;
            RoutesNum ++;
            TaskRoutes[RoutesNum][0] = 1;
            TaskRoutes[RoutesNum][1] = 0;
        } else{
            TaskRoutes[RoutesNum][0] ++;
            TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = Indi->Sequence[i];
        }
    }
    TaskRoutes[RoutesNum][0]++;
    TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;

    // NodeRoutes: each row is the actual node sequence of each sub-route
    for (i = 1; i <= RoutesNum; i++)
    {
        NodeRoutes[i][0] = 0;
        for (j = 1; j < TaskRoutes[i][0]; j++)
        {
            for (k = 1; k <= shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][0]; k++)
            {
                NodeRoutes[i][0] ++;
                NodeRoutes[i][NodeRoutes[i][0]] = shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][k];
            }
        }
    }

    // InRoutes: the row TID save all routes ID where task TID located
    // e.g. InRoutes[1] = {1, 2, 3}, task 1 exists in route 1, 2, 3.
    for (i = 1; i <= NRE+NRA; i++)
    {
        InRoutes[i][0] = 0;
    }
    for (i=1; i<=RoutesNum; i++)
    {
        memset(CheckMark, 0, sizeof(CheckMark));
        for (j=1; j < NodeRoutes[i][0]; j++)
        {
            int TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
            if (TID > NRE)
                TID -= NRE;
            if (TID == 0) // if TID==0, it is not a task
                continue;

            if (!CheckMark[TID])
            {
                CheckMark[TID] = 1;
                InRoutes[TID][0] ++;
                InRoutes[TID][InRoutes[TID][0]] = i;
            }
        }
    }

    // FreeTasks: tasks that exists in over one routes
    // tasks only exist in one route is ignored because it can't be assigned into different routes.
    int FreeTasks[NRE + NRA + 1];
    FreeTasks[0] = 0;
    for (i = 1; i <= NRE + NRA; i++)
    {
        if (InRoutes[i][0] == 1) // tasks only exist in one route;
            continue;
        FreeTasks[0] ++;
        FreeTasks[FreeTasks[0]] = i;
    }


    struct TaskAssignment
    {
        int Assignment[300]; // which route is assigned for each task.
        int Loads[101];
        int TotalVioLoad;
    };

    struct TaskAssignment CurrTA, NeighTA, NextTA, BestTA;

    memset(CurrTA.Assignment, 0, sizeof(CurrTA.Assignment));
    memset(CurrTA.Loads, 0, sizeof(CurrTA.Loads));
    CurrTA.Loads[0] = RoutesNum;

    for (i = 1; i <= NRE + NRA; i++)
    {
        if (InRoutes[i][0] == 1)
        {
            CurrTA.Assignment[i] = InRoutes[i][1];
            CurrTA.Loads[InRoutes[i][1]] += inst_tasks[i].demand;
        }
    }

    int LeftTasks[NRE+NRA+1], RouteCandDemands[RoutesNum+1], CandRoutes[NRE+NRA+1][101];
    AssignArray(FreeTasks, LeftTasks);
    memset(RouteCandDemands, 0, sizeof(RouteCandDemands));
    for(i = 1; i <= NRE+NRA; i++)
    {
        CandRoutes[i][0] = 0; // The routes which task i can be assigned.
        for (j = 1; j <= InRoutes[i][0]; j++)
        {
            if (CurrTA.Loads[InRoutes[i][j]] >= capacity)
                continue;

            CandRoutes[i][0] ++;
            CandRoutes[i][CandRoutes[i][0]] = InRoutes[i][j];
        }
    }

    // RouteCandDemands[i] denotes Route[i].
    // The value of RouteCandDemands[i] represents the total demand of all possible assigned tasks.
    for (i = 1; i <= LeftTasks[0]; i++)
    {
        for (j = 1; j <= InRoutes[LeftTasks[i]][0]; j++)
        {
            RouteCandDemands[InRoutes[LeftTasks[i]][j]] += inst_tasks[LeftTasks[i]].demand;
        }
    }

    int SelID, AssignedRouteID;
    int n;
    for (n = 1; n <= FreeTasks[0]; n++)
    {
        SelID = 0; // select item
        for (i = 1; i <= LeftTasks[0]; i++)
        {
            if (SelID == 0)
            {
                SelID = i;
            } else if (CandRoutes[LeftTasks[i]][0] < CandRoutes[LeftTasks[SelID]][0]) // select smallest _|^|_
            {
                SelID = i;
            } else if (CandRoutes[LeftTasks[i]][0] == CandRoutes[LeftTasks[SelID]][0])
            {
                if (inst_tasks[LeftTasks[i]].demand > inst_tasks[LeftTasks[SelID]].demand) // select the max demand
                {
                    SelID = i;
                }
            }
        }

        AssignedRouteID = 0; // select bin
        for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
        {
            if (AssignedRouteID == 0)
            {
                AssignedRouteID = i;
            }
            else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand <
                     CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
            {
                AssignedRouteID = i;
            }
            else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand ==
                     CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
            {
                if (RouteCandDemands[InRoutes[LeftTasks[SelID]][i]]+CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]] >
                    RouteCandDemands[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+
                    CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]])
                {
                    AssignedRouteID = i;
                }
            }
        }

        CurrTA.Assignment[LeftTasks[SelID]] = InRoutes[LeftTasks[SelID]][AssignedRouteID];
        CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] += inst_tasks[LeftTasks[SelID]].demand;
        for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
        {
            RouteCandDemands[InRoutes[LeftTasks[SelID]][i]] -= inst_tasks[LeftTasks[SelID]].demand;
        }

        if (CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] >= capacity)
        {
            for (i = 1; i <= NRE+NRA; i++)
            {
                for (j = 1; j <= CandRoutes[i][0]; j++)
                {
                    if (CandRoutes[i][j] == CurrTA.Assignment[LeftTasks[SelID]])
                    {
                        delete_element(CandRoutes[i], j);
                        break;
                    }
                }
            }
        }
        delete_element(LeftTasks, SelID);
    }
    CurrTA.TotalVioLoad = 0;
    for (i = 1; i <= RoutesNum; i++)
    {
        if (CurrTA.Loads[i] > capacity)
            CurrTA.TotalVioLoad += CurrTA.Loads[i]-capacity;
    }

    BestTA = CurrTA;

    // tabu search for repairing
    int TabuList[NRE+NRA+1];
    int TabuTenure = FreeTasks[0]/2;
    int TabuTask, Tabu;
    memset(TabuList, 0, sizeof(TabuList));

    int count = 0;
    int stlcount = 0;
    while (BestTA.TotalVioLoad > 0)
    {
        count ++;
        stlcount ++;
        NextTA.TotalVioLoad = INF;
        for (i = 1; i <= FreeTasks[0]; i++)
        {
            for (j = 1; j <= InRoutes[FreeTasks[i]][0]; j++)
            {
                if (InRoutes[FreeTasks[i]][j] == CurrTA.Assignment[FreeTasks[i]])
                    continue;

                NeighTA = CurrTA;
                NeighTA.Assignment[FreeTasks[i]] = InRoutes[FreeTasks[i]][j];
                NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] -= inst_tasks[FreeTasks[i]].demand;
                NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] += inst_tasks[FreeTasks[i]].demand;

                if (CurrTA.Loads[NeighTA.Assignment[FreeTasks[i]]] >= capacity)
                {
                    NeighTA.TotalVioLoad += inst_tasks[FreeTasks[i]].demand;
                }
                else if (NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] > capacity)
                {
                    NeighTA.TotalVioLoad += NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]]-capacity;
                }

                if (NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] >= capacity)
                {
                    NeighTA.TotalVioLoad -= inst_tasks[FreeTasks[i]].demand;
                }
                else if (CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]] > capacity)
                {
                    NeighTA.TotalVioLoad -= CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]]-capacity;
                }

                Tabu = 0;
                if (TabuList[FreeTasks[i]] > 0 && NeighTA.TotalVioLoad >= BestTA.TotalVioLoad)
                    Tabu = 1;

                if (Tabu)
                    continue;

                if (NeighTA.TotalVioLoad < NextTA.TotalVioLoad)
                {
                    NextTA = NeighTA;
                    TabuTask = FreeTasks[i];
                }
            }
        }
        for (i = 1; i <= NRE + NRA; i++)
        {
            if (TabuList[i] > 0)
                TabuList[i] --;
        }

        TabuList[TabuTask] = TabuTenure;
        CurrTA = NextTA;
        if (CurrTA.TotalVioLoad < BestTA.TotalVioLoad)
        {
            stlcount = 0;
            BestTA = CurrTA;
        }

        if (count == 2*(NRA+NRA) || stlcount == (NRE + NRA) / 2)
            break;
    }
    if (BestTA.TotalVioLoad > Indi->TotalVioLoad)
        return;

    int Served[NRA + NRE + 1];
    memset(Served, 0, sizeof(Served));

    Indi->Sequence[0] = 1;
    Indi->Sequence[1] = 0;

    for (i=1; i <= RoutesNum; i++)
    {
        for (j = 1; j < NodeRoutes[i][0]; j++)
        {
            int TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
            int TmpTID = TID;

            if (TmpTID > NRE)
                TmpTID -= NRE;

            if (TmpTID == 0)
                continue;

            if (BestTA.Assignment[TmpTID] == i && !Served[TmpTID])
            {
                Served[TmpTID] = 1;
                Indi->Sequence[0] ++;
                Indi->Sequence[Indi->Sequence[0]] = TID;
            }

        }
        if (Indi->Sequence[Indi->Sequence[0]] != 0)
        {
            Indi->Sequence[0] ++;
            Indi->Sequence[Indi->Sequence[0]] = 0;
        }
    }

    for(i = BestTA.Loads[0]; i > 0; i--)
    {
        if (BestTA.Loads[i] == 0)
            delete_element(BestTA.Loads, i);
    }

    Indi->TotalCost = get_task_seq_total_cost(Indi->Sequence, inst_tasks);
    AssignArray(BestTA.Loads, Indi->Loads);
    Indi->TotalVioLoad = BestTA.TotalVioLoad;

}



























