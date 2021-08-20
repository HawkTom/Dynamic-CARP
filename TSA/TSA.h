//
// Created by hao on 13/06/2020.
//

#ifndef CARP_TSA_H
#define CARP_TSA_H

# include "../functions.h"
# include "../heutistic.h"
# include <math.h>

int SingleInsertion(Individual *BestSINeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness );
int DoubleInsertion(Individual *BestDINeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness);
int SWAP(Individual *BestSWAPNeighbor, Individual *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness);
void RepairInfeasibility (Individual *Indi, const Task *inst_tasks);



#endif //CARP_TSA_H
