#include "timestamp.h"

#include <string.h>

#include "bsp.h"

static timestamp_t s_timestamp = {0};

static volatile uint8_t run = 0;
static uint8_t timestampBuffer[6];

// Since 32768/1000 = 32.768 we need to use counter values 33,33,33,32 = 32750.
// And when the second is full we add 32768 - 32750 - 1 = 17 + 33 = 50
static uint8_t millisecondCorrectionCounter = 1;

void timestampTick()
{
	if (!run)
		return;

	if (++s_timestamp.milliseconds >= 1000)
	{
		s_timestamp.seconds++;
		s_timestamp.milliseconds = 0;

		millisecondCorrectionCounter = 1;
		TA0CCR1 += 50;
		return;
	}

	if (millisecondCorrectionCounter++ >= 4)
	{
		millisecondCorrectionCounter = 1;
		TA0CCR1 += 32;
	}
	else
	{
		TA0CCR1 += 33;
	}
}

void timestampInit(uint32_t timeT)
{
	s_timestamp.seconds = timeT;
	s_timestamp.milliseconds = 0;
	millisecondCorrectionCounter = 1;

    uint16_t valueOffset = 0;

    while (valueOffset != TA0R)
        valueOffset = TA0R;

    valueOffset += 32;

    TA0CCR1 = valueOffset;

    TA0CCTL1 &= ~CCIFG;
    TA0CCTL1 |= CCIE;

	run = 1;
}

uint8_t* timestampAsBuffer()
{
	bspIState_t intState;

	BSP_ENTER_CRITICAL_SECTION(intState);
	timestamp_t timestampTemp = s_timestamp;
	BSP_EXIT_CRITICAL_SECTION(intState);

	memcpy(timestampBuffer, &timestampTemp.seconds, 4);
	memcpy(timestampBuffer + 4, &timestampTemp.milliseconds, 2);

	return timestampBuffer;
}



