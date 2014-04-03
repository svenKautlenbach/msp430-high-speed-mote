#pragma once

#include <stdint.h>

typedef struct
{
	uint32_t seconds;
	uint16_t milliseconds;
}timestamp_t;

void timestampTick();
void timestampInit(uint32_t timeT);
uint8_t* timestampAsBuffer();
