/**
 * @file mimo.c
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief MMWave Radar in MIMO setup
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ti/mmwave/mmwave.h"


/**
 * @brief Routine to close trace file
 * 
 */
FILE* rls_traceF = NULL;
void CloseTraceFile() {
  if (rls_traceF != NULL) {
    fclose(rls_traceF);
    rls_traceF = NULL;
  }
}


/**
 * @brief Application entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main (int argc, char *argv[]) {
  printf("MMWave MIMO Application\n");
  return 0;
}
