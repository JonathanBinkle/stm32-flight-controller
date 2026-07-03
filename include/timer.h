#ifndef FC_TIMER_H
#define FC_TIMER_H

/*
 * Microsecond timer.
 */

#include <stdint.h>

void timer_setup(void);

/* Delay in microseconds. */
void timer_wait_us(uint32_t us);

/* Current microsecond timestamp. Wraps every ~71 minutes. */
uint32_t timer_now_us(void);

/* Block until the specified absolute timestamp. */
void timer_wait_until_us(uint32_t us);

#endif /* FC_TIMER_H */
