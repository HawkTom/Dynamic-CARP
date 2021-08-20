//
// Created by hao on 13/06/2020.
//


#include "TSA.h"
#include <math.h>


int check_cost(Individual solution, const Task *inst_tasks);
void indi_route_converter3(Individual *dst, Individual *src, const Task *inst_tasks);

void TSA(const Task *inst_tasks, Individual *TSASolution)
{
    int i, j,k, m;

    int NVer, NRE, NRA, NNR, NVeh, Cap, LLB;
    NVer = vertex_num;
    NRE = req_edge_num;
    NRA = req_arc_num;
    NNR = 2*nonreq_edge_num+nonreq_arc_num;
    NVeh = vehicle_num;
    Cap = capacity;
    LLB = 5566; // Lower bound can be obtained from some paper.


    Individual InitSolution;
    int ServMark[2*NRE+NRA+1]; // the mark for served tasks
    memset(ServMark, 1, sizeof(ServMark));
    path_scanning(&InitSolution, inst_tasks, ServMark);

    InitSolution.Fitness = InitSolution.TotalCost;

    // printf("TSA: initial is ok \n");

    // following five variables are defined for Frederick Heuristic
    int Route[MAX_TASK_SEQ_LENGTH], FHRoute[MAX_TASK_SEQ_LENGTH], Position[101], TmpSeq[MAX_TASK_SEQ_LENGTH], Cost1, Cost2;

    TmpSeq[0] = 1; // tmp solution to link all processed sub-routes
    InitSolution.TotalCost = 0;

    // improve initial solution by Frederick Heuristic, 这一段原文没有

    find_ele_positions(Position, InitSolution.Sequence, 0);
    for(i = 1; i < Position[0]; i++)
    {
        AssignSubArray(InitSolution.Sequence, Position[i], Position[i+1], Route);
        FredericksonHeuristic(FHRoute, Route, inst_tasks);
        Cost1 =get_task_seq_total_cost(Route, inst_tasks);
        Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

        TmpSeq[0] --; // ??
        if (Cost1 < Cost2)
        {
            JoinArray(TmpSeq, Route); //link two routes
            InitSolution.TotalCost += Cost1;
        } else {
            JoinArray(TmpSeq, FHRoute);
            InitSolution.TotalCost += Cost2;
        }
    }

    InitSolution.Fitness = InitSolution.TotalCost;
    AssignArray(TmpSeq, InitSolution.Sequence);

    // For each required edge, which routes they located in.
    InitSolution.Assignment[0] = NRE + NRA; // record as the tabu list
    j = 1;
    for (i = 2; i <= InitSolution.Sequence[0]; i++)
    {
        if (InitSolution.Sequence[i] == 0)
        {
            j ++;
            continue;
        }
        int tmp = InitSolution.Sequence[i];
        if (tmp > NRE)
            tmp -= NRE;

        InitSolution.Assignment[tmp] = j; // record which routes each edge locates in.
    }

    Individual CurrSolution, BestSolution, BestFsbSolution, NextSolution, TmpSolution;
    int TabuList[2*NRE+NRA+1][101];
    memset(TabuList, 0, sizeof(TabuList));
    int TabuTenure = (NRE + NRA) / 2;

    CurrSolution = InitSolution;
    BestSolution = InitSolution;

    BestFsbSolution.TotalCost = 0;
    if (InitSolution.TotalVioLoad == 0)
    {
        BestFsbSolution = InitSolution;
    }

    double PenPrmt = 1; //penalty parameter
    int ConstFsb, ConstInFsb; // the number of iterations for feasible/unfeasible solution

    int FSI, FDI, FSWAP; //frequency of the different types of move

    ConstFsb = 0;
    ConstInFsb = 0;

    FSI = 1;
    FDI = 1; // 5 in original paper
    FSWAP = 1; // 5 in original paper

    int npi, mnpi, nti, mnti;

    mnpi = 900 * (int)sqrt(NRE + NRA); //

    npi = 0; // iteration
    nti = 0; //

    Individual BestSINeighbor, BestDINeighbor, BestSWAPNeighbor;
    int ChangedRoutesID[2], MovedTasks[3], BestMovedTasks[3], RID1, RID2;
    int TotalFitEval = 0, FitEval = 0, FitCost[40000][3];
    FitCost[0][0] = 0;
    // while (BestFsbSolution.TotalCost > LLB)

    
    clock_t start_t, finish_t;
    start_t = clock();
    double duration, duration1=0;

    

    // while (stop_iter < terminal_condition)
    while (npi < mnpi && duration < terminal_duration)
    {
        NextSolution.Fitness = INF;
        int improved = 0;

        if (npi % FSI == 0)
        {
            // single insertion
            FitEval = SingleInsertion(&BestSINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("Single Insertion, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestSINeighbor, inst_tasks);


            if (BestSINeighbor.TotalVioLoad == 0 && BestSINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        if (npi % FDI == 0)
        {
            // double insertion
            FitEval = DoubleInsertion(&BestDINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("Double Insertion, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestDINeighbor, inst_tasks);

            if (BestDINeighbor.TotalVioLoad == 0 && BestDINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestDINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestDINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestDINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        if (npi % FSWAP == 0)
        {
            // swap
            FitEval = SWAP(&BestSWAPNeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("SWAP, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestSWAPNeighbor, inst_tasks);

            if (BestSWAPNeighbor.TotalVioLoad == 0 && BestSWAPNeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSWAPNeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSWAPNeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSWAPNeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        npi ++;
        nti ++;

        if (NextSolution.TotalVioLoad == 0)
        {
            ConstFsb ++;
        } else{
            ConstInFsb ++;
        }

        // repair operator
        
        if (NextSolution.TotalCost < BestFsbSolution.TotalCost && NextSolution.TotalVioLoad > 0)
        {
            TmpSolution = NextSolution;
            RepairInfeasibility(&TmpSolution, inst_tasks);
            TotalFitEval ++;

            if(TmpSolution.TotalVioLoad < NextSolution.TotalVioLoad || (TmpSolution.TotalVioLoad == NextSolution.TotalVioLoad && TmpSolution.TotalCost < NextSolution.TotalCost))
            {
                NextSolution = TmpSolution;
            }

            if (NextSolution.TotalVioLoad == 0 && NextSolution.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = NextSolution;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
        }
        
        //
        if (NextSolution.TotalVioLoad == 0)
        {
            TmpSeq[0] = 1; // tmp solution to link all processed sub-routes
            NextSolution.TotalCost = 0;

            find_ele_positions(Position, NextSolution.Sequence, 0);
            for(i = 1; i < Position[0]; i++)
            {
                AssignSubArray(NextSolution.Sequence, Position[i], Position[i+1], Route);
                FredericksonHeuristic(FHRoute, Route, inst_tasks);
                Cost1 =get_task_seq_total_cost(Route, inst_tasks);
                Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

                TmpSeq[0] --;
                if (Cost1 < Cost2)
                {
                    JoinArray(TmpSeq, Route); //link two routes
                    NextSolution.TotalCost += Cost1;
                } else {
                    JoinArray(TmpSeq, FHRoute);
                    NextSolution.TotalCost += Cost2;
                }
            }

            NextSolution.Fitness = NextSolution.TotalCost;
            AssignArray(TmpSeq, NextSolution.Sequence);
        }
        check_cost(NextSolution, inst_tasks);
        

        if (improved && NextSolution.TotalVioLoad == 0)
        {
            BestFsbSolution = NextSolution;
            nti = 0;
        }

        NextSolution.Assignment[0] = req_edge_num + req_arc_num;
        j = 1;
        for (i = 2; i < NextSolution.Sequence[0]; i++)
        {
            if (NextSolution.Sequence[i] == 0)
            {
                j++;
                continue;
            }

            int tmp = NextSolution.Sequence[i];
            if ( tmp > req_edge_num)
                tmp -= req_edge_num ;

            NextSolution.Assignment[tmp] = j;

        }

        for (i = 1; i <= 2*req_edge_num + req_arc_num; i++)
        {
            for (j = 0; j <= vehicle_num; j++)
            {
                if (TabuList[i][j] > 0)
                    TabuList[i][j] --;
            }
        }

        // Assignment: each task belongs to which routes, 1 -> req_edge_num
        for (i = 1; i <= NextSolution.Assignment[0]; i++)
        {
            if (NextSolution.Assignment[i] == CurrSolution.Assignment[i])
                continue;

            int tmp = i;
            if (tmp > req_edge_num)
            {
                tmp += req_edge_num;
            }


            TabuList[tmp][CurrSolution.Assignment[i]] = TabuTenure;
            if ( i <= req_edge_num)
            {
                TabuList[i+req_edge_num][CurrSolution.Assignment[i]] = TabuTenure;
            }
        }

        if (NextSolution.Fitness < BestSolution.Fitness)
            BestSolution = NextSolution;

        CurrSolution = NextSolution;

        if (npi % 10 == 0)
        {
            if (ConstFsb == 10)
            {
                PenPrmt /= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            } else if (ConstInFsb == 10)
            {
                PenPrmt *= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            }
            // printf("PenPrmt: %f \n", PenPrmt);
            ConstFsb = 0;
            ConstInFsb = 0;
        }

        if (npi == mnpi)
            break;


        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        

    }
    printf("TSA: npi, %d, BestFsbCost = %d\n", npi, BestFsbSolution.TotalCost);

    // memcpy(TSASolution->Sequence, BestFsbSolution.Sequence, sizeof(BestFsbSolution.Sequence));
    // memcpy(TSASolution->Loads, BestFsbSolution.Loads, sizeof(BestFsbSolution.Loads));

    indi_route_converter3(TSASolution, &BestFsbSolution, inst_tasks);
    memcpy(TSASolution->Assignment, BestFsbSolution.Assignment, sizeof(BestFsbSolution.Assignment));
    TSASolution->TotalCost = BestFsbSolution.TotalCost;
    TSASolution->TotalVioLoad = BestFsbSolution.TotalVioLoad;
    TSASolution->Fitness = BestFsbSolution.Fitness;

}



void TSAih(const Task *inst_tasks, Individual *TSASolution, Individual InitSolution)
{
    int i, j,k, m;

    int NVer, NRE, NRA, NNR, NVeh, Cap, LLB;
    NVer = vertex_num;
    NRE = req_edge_num;
    NRA = req_arc_num;
    NNR = 2*nonreq_edge_num+nonreq_arc_num;
    NVeh = vehicle_num;
    Cap = capacity;
    LLB = 5566; // Lower bound can be obtained from some paper.

    InitSolution.Fitness = InitSolution.TotalCost;

    // printf("TSA: initial is ok \n");

    // following five variables are defined for Frederick Heuristic
    int Route[MAX_TASK_SEQ_LENGTH], FHRoute[MAX_TASK_SEQ_LENGTH], Position[101], TmpSeq[MAX_TASK_SEQ_LENGTH], Cost1, Cost2;

    TmpSeq[0] = 1; // tmp solution to link all processed sub-routes
    InitSolution.TotalCost = 0;

    // improve initial solution by Frederick Heuristic, 这一段原文没有

    find_ele_positions(Position, InitSolution.Sequence, 0);
    for(i = 1; i < Position[0]; i++)
    {
        AssignSubArray(InitSolution.Sequence, Position[i], Position[i+1], Route);
        FredericksonHeuristic(FHRoute, Route, inst_tasks);
        Cost1 =get_task_seq_total_cost(Route, inst_tasks);
        Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

        TmpSeq[0] --; // ??
        if (Cost1 < Cost2)
        {
            JoinArray(TmpSeq, Route); //link two routes
            InitSolution.TotalCost += Cost1;
        } else {
            JoinArray(TmpSeq, FHRoute);
            InitSolution.TotalCost += Cost2;
        }
    }

    InitSolution.Fitness = InitSolution.TotalCost;
    AssignArray(TmpSeq, InitSolution.Sequence);

    // For each required edge, which routes they located in.
    InitSolution.Assignment[0] = NRE + NRA; // record as the tabu list
    j = 1;
    for (i = 2; i <= InitSolution.Sequence[0]; i++)
    {
        if (InitSolution.Sequence[i] == 0)
        {
            j ++;
            continue;
        }
        int tmp = InitSolution.Sequence[i];
        if (tmp > NRE)
            tmp -= NRE;

        InitSolution.Assignment[tmp] = j; // record which routes each edge locates in.
    }

    Individual CurrSolution, BestSolution, BestFsbSolution, NextSolution, TmpSolution;
    int TabuList[2*NRE+NRA+1][101];
    memset(TabuList, 0, sizeof(TabuList));
    int TabuTenure = (NRE + NRA) / 2;

    CurrSolution = InitSolution;
    BestSolution = InitSolution;

    BestFsbSolution.TotalCost = 0;
    if (InitSolution.TotalVioLoad == 0)
    {
        BestFsbSolution = InitSolution;
    }

    double PenPrmt = 1; //penalty parameter
    int ConstFsb, ConstInFsb; // the number of iterations for feasible/unfeasible solution

    int FSI, FDI, FSWAP; //frequency of the different types of move

    ConstFsb = 0;
    ConstInFsb = 0;

    FSI = 1;
    FDI = 1; // 5 in original paper
    FSWAP = 1; // 5 in original paper

    int npi, mnpi, nti, mnti;

    mnpi = 900 * (int)sqrt(NRE + NRA); //

    npi = 0; // iteration
    nti = 0; //



    clock_t start_t, finish_t;
    start_t = clock();
    double duration, duration1=0;

    Individual BestSINeighbor, BestDINeighbor, BestSWAPNeighbor;
    int ChangedRoutesID[2], MovedTasks[3], BestMovedTasks[3], RID1, RID2;
    int TotalFitEval = 0, FitEval = 0, FitCost[40000][3];
    FitCost[0][0] = 0;
    // while (BestFsbSolution.TotalCost > LLB)
    while (npi < mnpi && duration < terminal_duration)
    {
        NextSolution.Fitness = INF;
        int improved = 0;

        if (npi % FSI == 0)
        {
            // single insertion
            FitEval = SingleInsertion(&BestSINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("Single Insertion, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestSINeighbor, inst_tasks);


            if (BestSINeighbor.TotalVioLoad == 0 && BestSINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        if (npi % FDI == 0)
        {
            // double insertion
            FitEval = DoubleInsertion(&BestDINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("Double Insertion, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestDINeighbor, inst_tasks);

            if (BestDINeighbor.TotalVioLoad == 0 && BestDINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestDINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestDINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestDINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        if (npi % FSWAP == 0)
        {
            // swap
            FitEval = SWAP(&BestSWAPNeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
//            printf("SWAP, %d\n", FitEval);
            TotalFitEval += FitEval;
            check_cost(BestSWAPNeighbor, inst_tasks);

            if (BestSWAPNeighbor.TotalVioLoad == 0 && BestSWAPNeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSWAPNeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSWAPNeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSWAPNeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        npi ++;
        nti ++;

        if (NextSolution.TotalVioLoad == 0)
        {
            ConstFsb ++;
        } else{
            ConstInFsb ++;
        }

        // repair operator
        
        // if (NextSolution.TotalCost < BestFsbSolution.TotalCost && NextSolution.TotalVioLoad > 0)
        // {
        //     TmpSolution = NextSolution;
        //     RepairInfeasibility(&TmpSolution, inst_tasks);
        //     TotalFitEval ++;

        //     if(TmpSolution.TotalVioLoad < NextSolution.TotalVioLoad || (TmpSolution.TotalVioLoad == NextSolution.TotalVioLoad && TmpSolution.TotalCost < NextSolution.TotalCost))
        //     {
        //         NextSolution = TmpSolution;
        //     }

        //     if (NextSolution.TotalVioLoad == 0 && NextSolution.TotalCost < BestFsbSolution.TotalCost)
        //     {
        //         improved = 1;
        //         BestFsbSolution = NextSolution;
        //         FitCost[0][0]++;
        //         FitCost[FitCost[0][0]][1] = TotalFitEval;
        //         FitCost[FitCost[0][0]][0] = npi;
        //         FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
        //     }
        // }
        // check_cost(BestFsbSolution, inst_tasks);
        
        //
        if (NextSolution.TotalVioLoad == 0)
        {
            TmpSeq[0] = 1; // tmp solution to link all processed sub-routes
            NextSolution.TotalCost = 0;

            find_ele_positions(Position, NextSolution.Sequence, 0);
            for(i = 1; i < Position[0]; i++)
            {
                AssignSubArray(NextSolution.Sequence, Position[i], Position[i+1], Route);
                FredericksonHeuristic(FHRoute, Route, inst_tasks);
                Cost1 =get_task_seq_total_cost(Route, inst_tasks);
                Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

                TmpSeq[0] --;
                if (Cost1 < Cost2)
                {
                    JoinArray(TmpSeq, Route); //link two routes
                    NextSolution.TotalCost += Cost1;
                } else {
                    JoinArray(TmpSeq, FHRoute);
                    NextSolution.TotalCost += Cost2;
                }
            }

            NextSolution.Fitness = NextSolution.TotalCost;
            AssignArray(TmpSeq, NextSolution.Sequence);
        }
        check_cost(NextSolution, inst_tasks);
        

        if (improved && NextSolution.TotalVioLoad == 0)
        {
            BestFsbSolution = NextSolution;
            nti = 0;
        }

        NextSolution.Assignment[0] = req_edge_num + req_arc_num;
        j = 1;
        for (i = 2; i < NextSolution.Sequence[0]; i++)
        {
            if (NextSolution.Sequence[i] == 0)
            {
                j++;
                continue;
            }

            int tmp = NextSolution.Sequence[i];
            if ( tmp > req_edge_num)
                tmp -= req_edge_num ;

            NextSolution.Assignment[tmp] = j;

        }

        for (i = 1; i <= 2*req_edge_num + req_arc_num; i++)
        {
            for (j = 0; j <= vehicle_num; j++)
            {
                if (TabuList[i][j] > 0)
                    TabuList[i][j] --;
            }
        }

        // Assignment: each task belongs to which routes, 1 -> req_edge_num
        for (i = 1; i <= NextSolution.Assignment[0]; i++)
        {
            if (NextSolution.Assignment[i] == CurrSolution.Assignment[i])
                continue;

            int tmp = i;
            if (tmp > req_edge_num)
            {
                tmp += req_edge_num;
            }


            TabuList[tmp][CurrSolution.Assignment[i]] = TabuTenure;
            if ( i <= req_edge_num)
            {
                TabuList[i+req_edge_num][CurrSolution.Assignment[i]] = TabuTenure;
            }
        }

        if (NextSolution.Fitness < BestSolution.Fitness)
            BestSolution = NextSolution;

        CurrSolution = NextSolution;

        if (npi % 10 == 0)
        {
            if (ConstFsb == 10)
            {
                PenPrmt /= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            } else if (ConstInFsb == 10)
            {
                PenPrmt *= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            }
            // printf("PenPrmt: %f \n", PenPrmt);
            ConstFsb = 0;
            ConstInFsb = 0;
        }

        if (npi == mnpi)
            break;

        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        // fp = fopen(path_trend, "a");
        // fprintf(fp, "%f, %d\t", (double)(finish_t - start_t) / CLOCKS_PER_SEC, InitSolution.TotalCost);
        // fclose(fp);
        

    }
    printf("TSAih: npi, %d, BestFsbCost = %d\n", npi, BestFsbSolution.TotalCost);

    // memcpy(TSASolution->Sequence, BestFsbSolution.Sequence, sizeof(BestFsbSolution.Sequence));
    // memcpy(TSASolution->Loads, BestFsbSolution.Loads, sizeof(BestFsbSolution.Loads));

    indi_route_converter3(TSASolution, &BestFsbSolution, inst_tasks);
    memcpy(TSASolution->Assignment, BestFsbSolution.Assignment, sizeof(BestFsbSolution.Assignment));
    TSASolution->TotalCost = BestFsbSolution.TotalCost;
    TSASolution->TotalVioLoad = BestFsbSolution.TotalVioLoad;
    TSASolution->Fitness = BestFsbSolution.Fitness;

}




void indi_route_converter3(Individual *dst, Individual *src, const Task *inst_tasks)
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




int check_cost(Individual solution, const Task *inst_tasks)
{
    int cost = get_task_seq_total_cost(solution.Sequence, inst_tasks);
    if (cost != solution.TotalCost)
    {
        return 0;
    }
     return 1;
}




































