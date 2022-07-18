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

typedef struct Event {
  uint8_t flag;
  pthread_cond_t cond;
  pthread_mutex_t mtx;
} EVENT;


/**
 * @brief Create an event
 *
 * @param flag Value oft the event's flag at creation
 */
EVENT event_create(uint8_t flag) {
  EVENT event;
  pthread_cond_init(&event.cond, NULL);
  pthread_mutex_init(&event.mtx, NULL);
  event.flag = flag;
  return event;
}


/**
 * @brief Initialize the event
 * 
 * @param event Pointer to the event to initialize
 */
void event_init(EVENT* event) {
  pthread_cond_init(&event->cond, NULL);
  pthread_mutex_init(&event->mtx, NULL);
  event->flag = 0;
}


/**
 * @brief Set the flag in the event structure
 * 
 * @param event Pointer to the event 
 */
void event_set(EVENT* event) {
  pthread_mutex_lock(&event->mtx);
  event->flag = 1;
  pthread_mutex_unlock(&event->mtx);
  pthread_cond_signal(&event->cond);
}


/**
 * @brief Wait for an event to be triggered
 * 
 * @param event Pointer to the event to watch
 */
void event_wait(EVENT* event) {
  pthread_mutex_lock(&event->mtx);
  while (!event->flag) pthread_cond_wait(&event->cond, &event->mtx);
  event->flag = 0;
  pthread_mutex_unlock(&event->mtx);
}

/**
 * @brief Initialize the event
 * 
 * @param event Pointer to the event to destroy
 */
void event_destroy(EVENT* event) {
  pthread_cond_destroy(&event->cond);
  pthread_mutex_destroy(&event->mtx);
  event->flag = 0;
}

#ifdef __cplusplus
  }
#endif

#endif
