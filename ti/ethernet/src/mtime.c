/**
 * @file mtime.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief Time related functions
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "mtime.h"

/**
 * @brief Delay on millisecond
 * 
 * @param delay Delay duration in ms
 */
unsigned int msleep(unsigned long delay) {
  struct timespec ts;
  unsigned int status;

  ts.tv_sec = delay / 1000;
  ts.tv_nsec = (delay % 1000) * 1000000;
  do { status = nanosleep(&ts, &ts); } while (status && (errno == EINTR));
  return status;
}
