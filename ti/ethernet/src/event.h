/**
 * @file event.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief Module for handling events
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef EVENT_H
#define EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <stdint.h>
#include <time.h>

typedef struct Event {
  uint8_t flag;
  pthread_cond_t cond;
  pthread_mutex_t mtx;
} EVENT;


/**
 * Functions
 */
EVENT event_create(uint8_t flag);
void event_init(EVENT* event);
void event_set(EVENT* event);
void event_wait(EVENT* event);
int event_wait_timeout(EVENT* event, uint32_t timeout);
void event_destroy(EVENT* event);

#ifdef __cplusplus
  }
#endif

#endif
