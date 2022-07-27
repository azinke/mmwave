/*
 * rls_osi.c - mmWaveLink OS Callback Implementation on Linux
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Updated by: AMOUSSOU Zinsou Kenneth
 * @date: 07-19-2022
*/

#include "rls_osi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include "../ethernet/src/mtime.h"

#define SEM_NAME_SIZE 16


typedef void (*P_OS_CALLBACK_FUNCTION)(int32_t);
sem_t rls_globalLockMutexCounter;
osiLockObj_t* rls_pGloblaLockObj = NULL;


/**
 * @brief Delay function
 *
 * @param duration Duration of the delay in millisecond
 * @return int Return 0 to notify successful completion
 */
int osiSleep(uint32_t duration) {
  msleep(duration);
  return OSI_OK;
}

/*******************************************************************************

  SYNC OBJECT

********************************************************************************/

int osiSyncObjCreate(osiSyncObj_t* pSyncObj, char* pName) {
  // unsigned int pNameLen = strlen(pName);
  char sname[SEM_NAME_SIZE];
  sname[0] = '/';
  strcpy(sname+1, pName);

  if (pSyncObj == NULL) {
    return OSI_INVALID_PARAMS;
  }

  *pSyncObj = malloc(sizeof(sem_t));
  if (*pSyncObj == NULL) return OSI_OPERATION_FAILED;

  sem_unlink(sname);  // Unlink any existing semaphore with the same name
  *pSyncObj = sem_open(sname, O_CREAT | O_EXCL, 0466, 0);
  sem_unlink(sname);  // Unlink the semaphore as soon as it's destroyed

  if (*pSyncObj == SEM_FAILED) return OSI_OPERATION_FAILED;

  if (*pSyncObj == NULL)  {
    return OSI_OPERATION_FAILED;
  }
  return OSI_OK;
}


int osiSyncObjDelete(osiSyncObj_t* pSyncObj) {
  uint8_t RetVal;

  if ((pSyncObj == NULL) || (*pSyncObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }
  sem_close(*pSyncObj);
  sem_destroy(*pSyncObj);
  return OSI_OK;
}


int osiSyncObjSignal(osiSyncObj_t* pSyncObj) {
  if ((pSyncObj == NULL) || (*pSyncObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }

  int status = sem_post(*pSyncObj);
  return OSI_OK;
}


int osiSyncObjWait(osiSyncObj_t* pSyncObj , osiTime_t Timeout) {
  int RetVal;

  if ((pSyncObj == NULL) || (*pSyncObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += (Timeout / 1000);
  ts.tv_nsec += (Timeout % 1000) * 1000000;

  RetVal = sem_timedwait(*pSyncObj, &ts);
  if (RetVal == 0) return OSI_OK;
  else if (RetVal == ETIMEDOUT) return OSI_TIMEOUT;
  else return OSI_OPERATION_FAILED;
}


/*******************************************************************************

    LOCKING OBJECT

********************************************************************************/

int osiLockObjCreate(osiLockObj_t* pLockObj, char* pName) {

  if (NULL == pLockObj) {
    return OSI_INVALID_PARAMS;
  }
  *pLockObj = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t));
  if (*pLockObj == NULL) return OSI_OPERATION_FAILED;

  pthread_mutex_init(*pLockObj, NULL);

  if (strcmp(pName, "GlobalLockObj") == 0) {
    /* save reference to the global lock pointer */
    rls_pGloblaLockObj = pLockObj;

    /* reset the counter */
    sem_init(&rls_globalLockMutexCounter, 1, 0);
  }

  if (*pLockObj == NULL)  {
    return OSI_OPERATION_FAILED;
  }

  return OSI_OK;
}


int osiLockObjDelete(osiLockObj_t* pLockObj) {
  int32_t RetVal;

  if ((pLockObj == NULL) || (*pLockObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }

  /* if we are going to delete the "GlobalLock" then wait till all threads
  waiting on this mutex are released */
  if (rls_pGloblaLockObj == pLockObj) {
    int gLockCount;
    do {
      msleep(1);
      sem_getvalue(&rls_globalLockMutexCounter, &gLockCount);
    } while(gLockCount > 0);

    sem_destroy(&rls_globalLockMutexCounter);
    rls_pGloblaLockObj = NULL;
  }

  pthread_mutex_destroy(*pLockObj);
  free(*pLockObj);
  return OSI_OK;
}


int osiLockObjLock(osiLockObj_t* pLockObj , osiTime_t Timeout) {
  int32_t RetVal;
  if ((pLockObj == NULL) || (*pLockObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }

  /* Increment the global lock counter  */
  if (rls_pGloblaLockObj == pLockObj) {
    sem_post(&rls_globalLockMutexCounter);
  }

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += (Timeout / 1000);
  ts.tv_nsec += (Timeout % 1000) * 1000000;

  RetVal = pthread_mutex_timedlock(*pLockObj, &ts);

  /* Decrement the global lock counter  */
  if (rls_pGloblaLockObj == pLockObj) {
    sem_wait(&rls_globalLockMutexCounter);
  }

  if (RetVal == 0) return OSI_OK;
  else if (RetVal == ETIMEDOUT) return OSI_TIMEOUT;
  else return OSI_OPERATION_FAILED;
}


int osiLockObjUnlock(osiLockObj_t* pLockObj) {
  int32_t RetVal;
  if ((pLockObj == NULL) || (*pLockObj == NULL)) {
    return OSI_INVALID_PARAMS;
  }

  RetVal = pthread_mutex_unlock(*pLockObj);
  if (RetVal != 0) return OSI_OPERATION_FAILED;
  return OSI_OK;
}


/*******************************************************************************

    SPWAN and CONTEXTS

********************************************************************************/



typedef struct spawnThreadEntry {
  rlsSpawnEntryFunc_t entryFunc;
  const void*         pParam;
} spawnThreadEntry_t;


typedef struct spawnCB {
  pthread_t  ThreadHdl;
  pthread_t   ThreadID;
} spawnCB_t;

spawnCB_t* rls_pSpawnCB=NULL;

#define SPAWN_MESSAGE           (WM_APP + 328) // custom message for thread

int osiSpawn(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags) {
  pEntry(pValue);
  return 0;
}


void* ExecuteEntryFunc(void* lpParam)  {
  spawnThreadEntry_t* pTh = (spawnThreadEntry_t*)(lpParam);
  pTh->entryFunc(pTh->pParam);
  free(pTh);
  return NULL;
}


int osiExecute(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags) {
  pthread_t  ThreadHdl;
  spawnThreadEntry_t *te = (spawnThreadEntry_t*)malloc(sizeof(spawnThreadEntry_t));
  te->entryFunc = pEntry;
  te->pParam = pValue;
  pthread_create(&ThreadHdl, NULL, ExecuteEntryFunc, te);
  msleep(1);
  return 0;
}


unsigned long osiGetTime(void) {
  struct timespec ts;
  unsigned long clocktick = 0U;
  clock_gettime(CLOCK_REALTIME, &ts);
  clocktick  = ts.tv_nsec / 1000000;
  clocktick += ts.tv_sec * 1000;
  return clocktick;
}
