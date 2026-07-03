#ifndef FC_UTIL_H
#define FC_UTIL_H

/* Fatal error loop: blink LED and send message over USB. */
void error(const char *msg);

/* Linearly maps value from SRC_MIN..SRC_MAX to DST_MIN..DST_MAX. */
float map_into_range(float src_min, float src_max, float val, float dst_min,
                     float dst_max);

/* Clamps value to MIN..MAX (inclusive). */
float clamp(float val, float min, float max);

#endif /* FC_UTIL_H */
