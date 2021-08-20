#include <stdio.h>
#include "functions.h"
#include "dirent.h"
#include "heutistic.h"
#include "time.h"


int req_arc_num = 0; //NRA
int req_edge_num; //NRE
int nonreq_arc_num = 0;
int nonreq_edge_num;
int vertex_num;
int vehicle_num;
int capacity;
int task_num;
int total_arc_num;
int DEPOT;
int trav_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int serve_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int shortest_path[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];

int cost_backup[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
char map[20];

double remain_cap_ratio_lb = 0;
double remain_cap_ratio_ub = 0.5;

int terminal_condition = 20; //if the best has nevere been changed over 20 iterations, stop the algorithm
int terminal_duration = 180;
int debug = 0;

char * whichMap(const char *name, int k);
void get_first_solution(Individual *solution, const Task *inst_tasks);
int get_additional_cost(Vehicles state);
void saveInfo(const Individual solution, const char *map1, const int step, const int seed);
int loadInfo(Individual *solution, const char *map1, const int req_step);
void copy_individual(Individual *dest, Individual *src);


int main(int argc, char *argv[])
{
    experiment1(argc, argv);
    return 0;
}

void experiment3(int argc, char *argv[])
{
    
    printf("experiment3\n");
    char path[30];
    char path_trend[30];

    // strcpy(map, "egl/egl-s4-C.dat");
    // strcpy(path, "../result/egl/egl-s4-C.dat");
    // strcpy(path_trend, "../trend/egl/egl-s4-C.dat");

    sprintf(map, "egl/%s", argv[1]);
    sprintf(path, "../result/egl/%s", argv[1]);
    sprintf(path_trend, "../trend/egl/%s", argv[1]); 

    printf("Starting: %s\n", map);

    if (map[8] == 101)
    {
        terminal_duration = 60;
    } else
    {
        terminal_duration = 180;
    }

    FILE *fp;
    fp = fopen(path, "w");
    if (fp == NULL)
    {
        printf("can't open %s\n", path);
    }
    fprintf(fp, "%s\n", map);
    fclose(fp);

    FILE *fp_trend;
    fp_trend = fopen(path_trend, "w");
    if (fp_trend == NULL)
    {
        printf("can't open %s\n", path_trend);
    }
    fprintf(fp_trend, "%s\n", map);
    fclose(fp_trend);

    struct tm *tm_now;
    time_t now;

    remain_cap_ratio_lb = 0.5;
    remain_cap_ratio_ub = 1;
    

    // int seed = time(NULL);
    int seed = 1602922520;
    srand(seed);
    
    //  read map
    Task inst_tasks[MAX_TASKS_TAG_LENGTH];
    Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
    Arc inst_arcs[MAX_ARCS_TAG_LENGTH];
    Vehicles state;

    readMap(inst_tasks, inst_arcs, map);
    readMap(inst_tasks_vt, inst_arcs, map);

    Individual firstSolution;
    Individual *executeSolution;

    Individual TSASolution, TSASolution1, LMASolution, LMASolution1, MAENSolution, MAENSolution1;
    Individual MASDCvtSolution, MASDCvtSolution1, MASDCSolution;
    
    for (int instance = 0; instance <= 5; instance ++)
    {
        // processing for new graph:  update cost and construct virtual task
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();

        if (instance == 0)
        {
            get_first_solution(&firstSolution, inst_tasks);
            executeSolution = &firstSolution;
            goto change;
        }
        executeSolution->TotalCost = INF;
        construct_virtual_task(inst_tasks, inst_tasks_vt, state.stop, state.remain_capacity);
        // get_first_solution(&firstSolution, inst_tasks_vt);

        // calculate additional cost
        int additional_cost = get_additional_cost(state);

        // inherience a solution
        Individual inhrSolution;
        int remain_seq_num;
        remain_seq_num = repair_solution_greedy_insertion(&inhrSolution, state.remain_seqs, state.stop, inst_tasks_vt);
        
        inhrSolution.TotalCost = split(inhrSolution.Sequence, inhrSolution.Assignment, inst_tasks_vt);
        memset(inhrSolution.Loads, 0, sizeof(inhrSolution.Loads));
        if(!check_task_valid(inhrSolution.Sequence))
        {
            printf("Inhrsolution sequence not valid. \n");
            exit(0);
        }
        if (inhrSolution.TotalCost != get_task_seq_total_cost(inhrSolution.Sequence, inst_tasks_vt))
        {
            printf("Inhrsolution cost not valid. \n");
            exit(0);
        }
        int load = 0;
        for (int i = 2; i <= inhrSolution.Sequence[0]; i++)
        {   
            if (inhrSolution.Sequence[i] == 0)
            {
                inhrSolution.Loads[0] ++;
                inhrSolution.Loads[inhrSolution.Loads[0]] = load; 
                load = 0;
            } else
            {
                load += inst_tasks_vt[inhrSolution.Sequence[i]].demand;
            }
        }

        
        fp_trend = fopen(path_trend, "a");
        fprintf(fp_trend, "instance:%d.\n", instance+1);
        fclose(fp_trend);
        

        fp = fopen(path, "a");
        fprintf(fp, "%s \n", map);
        fprintf(fp, "instance:%d,seed:%d,remain_seq_num:%d,additional_cost:%d,V:%d,R:%d,OV:%d.\n", instance, seed, remain_seq_num, additional_cost, vertex_num, req_edge_num, state.stop[0]);
        fclose(fp);
        int seed2;
        for (int run = 1; run <= 25; run++)
        {
            now = time(NULL);
            tm_now = localtime(&now);
            printf("start datetime: %d-%d-%d %d:%d:%d\n",tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec) ;
            seed2 = (int)rand() + tm_now->tm_min * 100 * run + tm_now->tm_sec * run;

            printf("map: %s, instance: %d, run: %d, seed: %d seed2: %d \n", map, instance, run, seed, seed2);

            fp_trend = fopen(path_trend, "a");
            fprintf(fp_trend, "instance:%d,run:%d,seed2:%d.\n", instance, run, seed2);
            fclose(fp_trend);
            
            
            clear_solution(&TSASolution);
            clear_solution(&TSASolution1);
            clear_solution(&LMASolution);
            clear_solution(&LMASolution1);
            clear_solution(&MAENSolution);
            clear_solution(&MAENSolution1);
            clear_solution(&MASDCvtSolution);
            clear_solution(&MASDCvtSolution1);
            clear_solution(&MASDCSolution);

            srand(seed2);
            TSA(inst_tasks_vt, &TSASolution);
            if (executeSolution->TotalCost > TSASolution.TotalCost)
            {
                copy_individual(executeSolution, &TSASolution);
            }
            check_solution_valid(TSASolution, inst_tasks_vt);


            srand(seed2);
            TSAih(inst_tasks_vt, &TSASolution1, inhrSolution);
            if (executeSolution->TotalCost > TSASolution1.TotalCost)
            {
                copy_individual(executeSolution, &TSASolution1);
            }
            check_solution_valid(TSASolution1, inst_tasks_vt);

            srand(seed2);
            LMA(inst_tasks_vt, &LMASolution);
            if (executeSolution->TotalCost > LMASolution.TotalCost)
            {
                copy_individual(executeSolution, &LMASolution);
            }
            check_solution_valid(LMASolution, inst_tasks_vt);

            
            srand(seed2);
            LMAih(inst_tasks_vt, &LMASolution1, inhrSolution);
            if (executeSolution->TotalCost > LMASolution1.TotalCost)
            {
                copy_individual(executeSolution, &LMASolution1);
            }
            check_solution_valid(LMASolution1, inst_tasks_vt);
            

            srand(seed2);
            MAENS(inst_tasks_vt, &MAENSolution);
            if (executeSolution->TotalCost > MAENSolution.TotalCost)
            {
                copy_individual(executeSolution, &MAENSolution);
            }
            check_solution_valid(MAENSolution, inst_tasks_vt);


            srand(seed2);
            MAENSih(inst_tasks_vt, &MAENSolution1, inhrSolution);
            if (executeSolution->TotalCost > MAENSolution1.TotalCost)
            {
                copy_individual(executeSolution, &MAENSolution1);
            }
            check_solution_valid(MAENSolution1, inst_tasks_vt);


            srand(seed2);
            MASDCvt(&MASDCvtSolution, inst_tasks_vt);
            if (executeSolution->TotalCost > MASDCvtSolution.TotalCost)
            {
                copy_individual(executeSolution, &MASDCvtSolution);
            }
            check_solution_valid(MASDCvtSolution, inst_tasks_vt);

            srand(seed2);
            MASDCvtih(&MASDCvtSolution1, inst_tasks_vt, inhrSolution);
            if (executeSolution->TotalCost > MASDCvtSolution1.TotalCost)
            {
                copy_individual(executeSolution, &MASDCvtSolution1);
            }
            check_solution_valid(MASDCvtSolution1, inst_tasks_vt);

            srand(seed2);
            req_edge_num = req_edge_num - state.stop[0];
            task_num = req_edge_num * 2;
            MASDC(&MASDCSolution, inst_tasks, state.stop, state.remain_capacity);
            check_seq_valid(MASDCSolution, inst_tasks);
            req_edge_num = req_edge_num + state.stop[0];
            task_num = req_edge_num * 2;
            if (executeSolution->TotalCost > MASDCSolution.TotalCost)
            {
                copy_individual(executeSolution, &MASDCSolution);
            }


            printf("----best: %d \n", executeSolution->TotalCost);

               
            fp = fopen(path, "a");
            fprintf(fp, "run:%d,seed2:%d.\n", run, seed2);
            fprintf(fp, "restart:TSA:%d,LMA:%d,MAENS:%d,MASDCvt:%d.\n", TSASolution.TotalCost, LMASolution.TotalCost, MAENSolution.TotalCost, MASDCvtSolution.TotalCost);
            fprintf(fp, "inherit:TSA:%d,LMA:%d,MAENS:%d,MASDCvt:%d.\n", TSASolution1.TotalCost, LMASolution1.TotalCost, MAENSolution1.TotalCost, MASDCvtSolution1.TotalCost);
            fprintf(fp, "MASDC:%d.\n", MASDCSolution.TotalCost);
            fclose(fp);
            printf("\n");
        }
        
        printf("\n >>> best solution: %d \n\n", executeSolution->TotalCost);
        
        change: 
        memset(state.stop, 0, sizeof(state.stop));
        memset(state.remain_capacity, 0, sizeof(state.remain_capacity));
        memset(state.remain_seqs, 0, sizeof(state.remain_seqs));
        seed = time(NULL) + 7654321 * instance;
        // seed= 1603355167; 
        saveInfo(*executeSolution, map, instance+1, seed);
        nextScenario(executeSolution, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);
        printf("map: %s, instance: %d from seed: %d\n", map, instance+1, seed);
    }
    
    printf("hello world !\n");
}


void experiment2(int argc, char *argv[])
{
    printf("experiment2\n");
    char path[30];

    // strcpy(map, "egl/egl-s4-C.dat");
    // strcpy(path, "../result/egl-s4-C.dat");


   sprintf(map, "egl/%s", argv[1]);
   sprintf(path, "../result2/%s", argv[1]);

   printf("name: %s \n", map);

    // run: ./dcarp egl-e3-B.dat
    // argv[1]: egl-e3-B.dat
    // map: egl/egl-e3-B.dat
    // path: results/egl-e3-B.dat

    if (map[8] == 101)
    {
        terminal_duration = 60;
    } else
    {
        terminal_duration = 180;
    }

    FILE *fp;
    fp = fopen(path, "w");
    fprintf(fp, "%s \n", map);
    fclose(fp);

    remain_cap_ratio_lb = 0.5;
    remain_cap_ratio_ub = 1;

    for (int kk=1; kk <= 3; kk++)
    {
        int seed = time(NULL);
        // seed = 1604492211;
        //  read map
        Task inst_tasks[MAX_TASKS_TAG_LENGTH];
        Arc inst_arcs[MAX_ARCS_TAG_LENGTH];

        readMap(inst_tasks, inst_arcs, map);
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();

        // initial solution for static graph
        Individual firstSolution;
        get_first_solution(&firstSolution, inst_tasks);

        printf("%d get first solution\n", seed);

        // execute first solution for static graph
        Vehicles state;
        memset(state.stop, 0, sizeof(state.stop));
        memset(state.remain_capacity, 0, sizeof(state.remain_capacity));
        Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
        readMap(inst_tasks_vt, inst_arcs, map);
        nextScenario(&firstSolution, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);

        fp = fopen(path, "a");
        // fprintf(fp, "%s \n", map);
        fprintf(fp, "V:%d,R:%d,OV:%d\n", vertex_num, req_edge_num, state.stop[0]);
        fclose(fp);


        // processing for new graph:  update cost and construct virtual task
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();
        construct_virtual_task(inst_tasks, inst_tasks_vt, state.stop, state.remain_capacity);
        
        // calculate additional cost
        int additional_cost = get_additional_cost(state);

        // independent runs
        for (int run = 1; run <= 25; run++)
        {
            int seed1 = time(NULL) + run*1234569;
            // seed1= 1605726976;
            srand(seed1);
            printf("seed1, %d", seed1);

            req_edge_num = req_edge_num - state.stop[0];
            task_num = 2 * req_edge_num;
            Individual solution1;
            MASDC(&solution1, inst_tasks, state.stop, state.remain_capacity);

            srand(seed1);
            req_edge_num = req_edge_num + state.stop[0];
            task_num = 2 * req_edge_num;
            Individual solution2;
            MASDCvt(&solution2, inst_tasks_vt);


            printf("run: %d, 1: %d, 2: %d, 0: %d. \n", run, solution1.TotalCost, solution2.TotalCost-additional_cost, additional_cost);

            fp = fopen(path, "a");
            fprintf(fp, "run:%d,seed:%d,1:%d,2:%d,0:%d\n", run, seed1, solution1.TotalCost, solution2.TotalCost-additional_cost,
                    additional_cost);
            fclose(fp);

        }
    }
}


void experiment1(int argc, char *argv[]) 
{
    printf("experiment1 new\n");

    char path[30];

    strcmp(argv[1], "egl-e1-A.dat");
    strcpy(map, "egl/egl-e1-A.dat");
    strcpy(path, "result/egl-e1-A.dat");
    int instance = 1;


    // sprintf(map, "egl/%s", argv[1]);
    // sprintf(path, "result/%s", argv[1]);
    // int instance = atoi(argv[2]);

    if (map[8] == 101)
    {
        terminal_duration = 60;
    } else
    {
        terminal_duration = 180;
    }
    

    
    printf("%s \n", path);
    FILE *fp;
    if (instance)
    {
        fp = fopen(path, "w");
    } else
    {
        fp = fopen(path, "a");
    }
    fprintf(fp, "%s \n", map);
    fclose(fp);


    struct tm *tm_now;
    time_t now;

    printf("%s\n", map);
    double ratios[5] = {4, 0, 0.33, 0.66, 1};
    int seed, seed1;
    for (int kk = instance; kk <= 3; kk++) {

        remain_cap_ratio_lb = ratios[kk];
        remain_cap_ratio_ub = ratios[kk+1];

        seed = time(NULL);
        now = time(NULL);
        tm_now = localtime(&now);
        printf("start datetime: %d-%d-%d %d:%d:%d\n",tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec) ;
        seed = (int)rand() + tm_now->tm_min * 100 * kk + tm_now->tm_sec * kk;
        
        //  read map
        Task inst_tasks[MAX_TASKS_TAG_LENGTH];
        Arc inst_arcs[MAX_ARCS_TAG_LENGTH];

        readMap(inst_tasks, inst_arcs, map);
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();

        // initial solution for static graph
        Individual firstSolution;
        get_first_solution(&firstSolution, inst_tasks);

//        printf("%d get first solution\n", seed);

        

        // setjmp(buf);
        // execute first solution for static graph
        Vehicles state;
        memset(state.stop, 0, sizeof(state.stop));
        memset(state.remain_capacity, 0, sizeof(state.remain_capacity));
        Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
        readMap(inst_tasks_vt, inst_arcs, map);


        // int seed = time(NULL);
        printf("instance: %d, seed: %d, remain_cap: ", kk, seed);
        // int seed = 1603081322;
        nextScenario(&firstSolution, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);

        for (int i=1; i <= state.remain_capacity[0]; i++)
        {
            printf("%d(%f) ", state.remain_capacity[i], state.remain_capacity[i]*1.0/capacity);
        }
        printf("\n");


        fp = fopen(path, "a");
        fprintf(fp, "%s \n", map);
        fprintf(fp, "seed:%d,V:%d,R:%d,OV:%d\n", seed, vertex_num, req_edge_num, state.stop[0]);
        fprintf(fp, "remain_capacity:");
        for (int i=1; i <= state.remain_capacity[0]; i++)
        {
            fprintf(fp, "%d,", state.remain_capacity[i]);
        }
        fprintf(fp, "\n");
        fclose(fp);

        // calculate additional cost
        int additional_cost = get_additional_cost(state);

        // processing for new graph:  update cost and construct virtual task
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();
        construct_virtual_task(inst_tasks, inst_tasks_vt, state.stop, state.remain_capacity);
        
        int independent;
        for (int run = 1; run <= 25; run++)
        {
            
            // int seed1 = time(NULL);
            seed1 = (int)rand();
            // seed1 =  1389806770;

            now = time(NULL);
            tm_now = localtime(&now);
            printf("%d:%d:%d -- ",tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);

            srand(seed1);
            // printf("seed1: %d\n", seed1);
            // generate solution by strategy 1
            int cost1 = 0;
            cost1 = back_and_new(state.stop, inst_tasks, seed1);

            now = time(NULL);
            tm_now = localtime(&now);
            printf("%d:%d:%d -- ",tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);

            srand(seed1);
            // optimize the new graph
            Individual solution2;
            LMA(inst_tasks_vt, &solution2);

            now = time(NULL);
            tm_now = localtime(&now);
            printf("%d:%d:%d\n ",tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);

            if (solution2.TotalCost != get_task_seq_total_cost(solution2.Sequence, inst_tasks_vt))
            {
                printf("solution2.cost error\n");
                exit(0);
            };
            independent = 0;
            for (int i=2; i < solution2.Sequence[0]; i++)
            {
                if (solution2.Sequence[i-1] == 0 && solution2.Sequence[i+1] == 0  && inst_tasks_vt[solution2.Sequence[i]].vt ==1)
                {
                    independent ++;
                }
            }

            // solution2.TotalCost = get_task_seq_total_cost(solution2.Sequence, inst_tasks_vt);
            printf("map: %s, run %d, 1: %d, 2: %d, 0: %d. \n", map, run, cost1, solution2.TotalCost-additional_cost, additional_cost);
            // printf("total: %d, independent: %d\n", state.remain_capacity[0], independent);

            // printf("%s \n", path);
            fp = fopen(path, "a");
            fprintf(fp, "run:%d,seed:%d,1:%d,2:%d,0:%d\n", run, seed1, cost1+additional_cost, solution2.TotalCost, additional_cost);
            fprintf(fp, "indepent_route:");
            fprintf(fp, "total:%d,independent:%d\n", state.remain_capacity[0], independent);
            fclose(fp);
        }
    }

    printf("Done!\n");
    return 0;
}

void copy_individual(Individual *dest, Individual *src)
{
    memcpy(dest->Sequence, src->Sequence, sizeof(src->Sequence));
    memcpy(dest->Loads, src->Loads, sizeof(src->Loads));
    dest->TotalCost = src->TotalCost;
}

void saveInfo(const Individual solution, const char *map1, const int instance, const int seed)
{
    char path[101];
    // strcpy(path, "../map/");
    strcpy(path, "../steps/");
    strcat(path, map1);

    FILE *fp;

    if (instance == 1)
    {
        fp = fopen(path, "w");
    } else
    {
        fp = fopen(path, "a");
    }
    fprintf(fp, "Instance: %d from seed: %d \n", instance, seed);
    for (int i = 0; i <= solution.Sequence[0]; i++)
    {
        fprintf(fp, "%d\t", solution.Sequence[i]);
    }
    fprintf(fp, "-1\n");
    for (int i = 0; i <= solution.Loads[0]; i++)
    {
        fprintf(fp, "%d\t", solution.Loads[i]);
    }
    fprintf(fp, "-1\n\n");
    fclose(fp);
}

int loadInfo(Individual *solution, const char *map1, const int req_step)
{
    char path[101];
    // strcpy(path, "../map/");
    strcpy(path, "../steps/");
    strcat(path, map1);

    FILE *fp;
    fp = fopen(path, "r");
    
    int seed, finished = 0;
    char dummy[101];
    int step;
    
    memset(solution->Sequence, 0, sizeof(solution->Sequence));
    memset(solution->Loads, 0, sizeof(solution->Loads));

    while (1)
    {
        fscanf(fp, "%s", dummy);
        if (strcmp(dummy, "Instance:")==0)
        {
            fscanf(fp, "%d", &step);
            if (step == req_step)
            {
                int num, index;
                fscanf(fp, "%s", dummy); // "from"
                fscanf(fp, "%s", dummy); // "seed"
                fscanf(fp, "%d", &seed); // seed
                index = 0;
                while (1)
                {
                    fscanf(fp, "%d", &num);
                    if (num >= 0)
                    {
                        solution->Sequence[index] = num;
                        index ++;
                    } else
                    {
                        break;
                    }
                }
                index = 0;
                while (1)
                {
                    fscanf(fp, "%d", &num);
                    if (num >= 0)
                    {
                        solution->Loads[index] = num;
                        index ++;
                    } else
                    {
                        finished = 1;
                        break;
                    }
                }                
            }
        }
        /* code */
        if (finished == 1)
        {
            break;
        }
    }
    fclose(fp);
    return seed;
    
}




void get_first_solution(Individual *solution, const Task *inst_tasks)
{
    int ServMark[2*req_edge_num+1]; // the mark for served tasks
    memset(ServMark, 1, sizeof(ServMark));
    path_scanning(solution, inst_tasks, ServMark);
}

int get_additional_cost(Vehicles state)
{
    int i, add_cost = 0;
    for (i = 1; i <= state.stop[0]; i++)
    {
        add_cost += min_cost[state.stop[i]][DEPOT];
    }

    return add_cost;
}

char * whichMap(const char *name, int k)
{
    static char files[101][101];
    static int flag = 1;
    static int num = 0;


    int i = 0;
    if (flag)
    {
        DIR *d;
        struct dirent *dir;
        d = opendir(name);
        if (d)
        {
            while ((dir = readdir(d)) != NULL)
            {
                strcpy(files[++i], "egl/");
                strcat(files[i], dir->d_name);
                num ++;
            }
            closedir(d);
        }
        flag = 0;
    }

    if (k > num)
    {
        return NULL;
    }
//    strcpy(files[k], "egl/egl-e1-B.dat");

    return files[k];

}

void loadDGraph(Task *inst_tasks, Arc *inst_arcs, Vehicles *state, const char *map1)
{
    FILE *fp;

    char dummy[101];
    int outv;


//    fp = fopen("../instances/example.dat", "r");
    char path[101];
    strcpy(path, "../map/");
    strcat(path, map1);
    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }

    while (fscanf(fp, "%s", dummy) != EOF)
    {

        if (strcmp(dummy, "vertices")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vertex_num);
            printf("vertex_num: %d \n", vertex_num);
        }
        else if (strcmp(dummy, "req_edge_num") == 0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &req_edge_num);
            printf("req_edge_num: %d \n", req_edge_num);
        }
        else if (strcmp(dummy, "nonreq_edge_num")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &nonreq_edge_num);
            printf("nonreq_edge_num: %d \n", nonreq_edge_num);
        }
        else if (strcmp(dummy, "vehicles")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vehicle_num);
            printf("vehicle_num: %d \n", vehicle_num);
        }
        else if (strcmp(dummy, "outside_vehicles")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &outv);
            state->stop[0] = outv;
            state->remain_capacity[0] = outv;
            printf("out vehicles: %d\n", outv);
        }
        else if (strcmp(dummy, "stop_point")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i = 1; i <= outv; i++)
            {
                fscanf(fp, "%d", &state->stop[i]);
                printf("%d ", state->stop[i]);
            }
            printf("\n");
        }
        else if (strcmp(dummy, "capacity")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &capacity);
        }
        else if (strcmp(dummy, "remaining_capacity")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i = 1; i <= outv; i++)
            {
                fscanf(fp, "%d", &state->remain_capacity[i]);
                printf("%d ", state->remain_capacity[i]);
            }
            printf("\n");
        }
        else if (strcmp(dummy, "remaining_tasks")==0)
        {
            fscanf(fp, "%s", dummy);
            memset(state->remain_seqs, 0, sizeof(state->remain_seqs));
            int i = 0;
            while (1)
            {
                fscanf(fp, "%d ", &state->remain_seqs[i]);
                printf("%d ", state->remain_seqs[state->remain_seqs[0]]);
                if (i == state->remain_seqs[0])
                {
                    break;
                }
                i++;
            }

        }
        else if (strcmp(dummy, "Edges")==0) {

            int node1, node2, cost, demand, index;


            fscanf(fp, "%s", dummy);
            task_num = 2 * req_edge_num + req_arc_num;
            total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;


            int u=0, v=task_num;
            for (int i = 1; i <= req_edge_num+nonreq_edge_num; i++) {
                fscanf(fp, "(%d,", &node1);
                fscanf(fp, "%d),", &node2);
                fscanf(fp, "%s:", dummy);
                fscanf(fp, "%d,", &cost);
                fscanf(fp, "%s:", dummy);
                fscanf(fp, "%d,", &demand);
                fscanf(fp, "%s:", dummy);
                fscanf(fp, "%d,", &index);
                printf("node1:%d node2:%d cost:%d demand:%d index:%d \n", node1, node2, cost, demand, index);
                if (demand > 0)
                {
                    u++;
                    inst_tasks[u].head_node = node1;
                    inst_tasks[u].tail_node = node2;
                    inst_tasks[u].serv_cost = cost;
                    inst_tasks[u].dead_cost = cost;
                    inst_tasks[u].demand = demand;
                    inst_tasks[u].inverse = u + req_edge_num;
                    inst_tasks[u].vt = 0;

                    inst_tasks[u + req_edge_num].head_node = inst_tasks[u].tail_node;
                    inst_tasks[u + req_edge_num].tail_node = inst_tasks[u].head_node;
                    inst_tasks[u + req_edge_num].dead_cost = inst_tasks[u].dead_cost;
                    inst_tasks[u + req_edge_num].serv_cost = inst_tasks[u].serv_cost;
                    inst_tasks[u + req_edge_num].demand = inst_tasks[u].demand;
                    inst_tasks[u + req_edge_num].inverse = u;
                    inst_tasks[u + req_edge_num].vt = 0;

                    inst_arcs[u].head_node = inst_tasks[u].head_node;
                    inst_arcs[u].tail_node = inst_tasks[u].tail_node;
                    inst_arcs[u].trav_cost = inst_tasks[u].dead_cost;

                    inst_arcs[u+req_edge_num].head_node = inst_arcs[u].tail_node;
                    inst_arcs[u+req_edge_num].tail_node = inst_arcs[u].head_node;
                    inst_arcs[u+req_edge_num].trav_cost = inst_arcs[u].trav_cost;

                } else {
                    v++;
                    inst_arcs[v].head_node = node1;
                    inst_arcs[v].tail_node = node2;
                    inst_arcs[v].trav_cost = cost;

                    inst_arcs[v+nonreq_edge_num].head_node = node2;
                    inst_arcs[v+nonreq_edge_num].tail_node = node1;
                    inst_arcs[v+nonreq_edge_num].trav_cost = cost;
                }
                if (index != 0)
                {
                    int idx;
                    if (index > 10000)
                    {
                        idx = index - 10000;
                        state->remain_seqs[idx] = u + req_edge_num;
                        continue;
                    }

                    if (index < 0)
                    {
                        if (index < -10000)
                        {
                            idx = index * -1 - 10000;
                        } else {
                            idx = index * -1;
                        }
                        state->remain_seqs[idx] = -1;
                        continue;
                    }

                    idx = index;
                    state->remain_seqs[idx] = u;
                }

            }
        }
    }

    fclose(fp);

    DEPOT = 1;
    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;

}