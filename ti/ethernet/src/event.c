/**
 * @file event.c
 * @author AMOUSSOU Z. Kenneth (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "event.h"

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
 * @brief Wait for an event to be triggered
 * 
 * @param event Pointer to the event to watch
 * @param timeout timeout in ms
 */
int event_wait_timeout(EVENT* event, uint32_t timeout) {
  pthread_mutex_lock(&event->mtx);
  uint32_t status;
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  t.tv_sec += timeout/1000;

  while (!event->flag) {
    status = pthread_cond_timedwait(&event->cond, &event->mtx, &t);
  }
  event->flag = 0;
  pthread_mutex_unlock(&event->mtx);
  if (status != 0) return -1;
  return 0;
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
