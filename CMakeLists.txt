cmake_minimum_required(VERSION 3.10)
project(dcarp_experiment1 C)

set(CMAKE_C_STANDARD 99)

add_executable(dcarp3 main.c functions.c functions.h heuristic.c heutistic.h readmap.c simulator.c simulator.h arrayoperations.c MAENS/MAENS.c MAENS/MAENS.h MAENS/operators.c MASDC/MASDC.c MASDC/MASDC.h MASDC/MASDCop.c MASDC/MASDCvt.c TSA/TSA.c TSA/TSA.h TSA/tsals.c LMA/LMA.c LMA/lmals.c LMA/LMAop.c)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}) 

# The followed line is for <math.h>
target_link_libraries(dcarp3 PRIVATE m)