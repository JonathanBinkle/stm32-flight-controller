#include "telemetry/telemetry.h"

static uint32_t seqnum = 0;

void telemetry_tick(void)
{
    seqnum++;
}

uint32_t telemetry_get_seqnum(void)
{
    return seqnum;
}
