/**********************************************************************************************
 * Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.
 *
 * IMPORTANT: Your use of this Software is limited to those specific rights granted under
 * the terms of a software license agreement between the user who downloaded the software,
 * his/her employer (which must be your employer) and Texas Instruments Incorporated (the
 * "License"). You may not use this Software unless you agree to abide by the terms of the
 * License. The License limits your use, and you acknowledge, that the Software may not be
 * modified, copied or distributed unless embedded on a Texas Instruments microcontroller
 * or used solely and exclusively in conjunction with a Texas Instruments radio frequency
 * transceiver, which is integrated into your product. Other than for the foregoing purpose,
 * you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
 * perform, display or sell this Software and/or its documentation for any purpose.
 *
 * YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
 * WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
 * WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 * NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
 * THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
 * INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
 * DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
 * THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 * Should you have any questions regarding your right to use this Software,
 * contact Texas Instruments Incorporated at www.TI.com.
 **************************************************************************************************/

// *************************************************************************************************
// Include section
#include <stdlib.h>
#include <string.h>

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "simpliciti.h"
#include "radio.h"

#include "project.h"

#include "battery.h"
#include "temperature.h"
#include "timestamp.h"


// *************************************************************************************************
// Defines section
#define TIMEOUT                                 (10u)

// Conversion from msec to ACLK timer ticks
#define CONV_MS_TO_TICKS(msec)                          (((msec) * 32768) / 1000)

// *************************************************************************************************
// Prototypes section


// *************************************************************************************************
// Extern section
extern uint8_t sInit_done;

// SimpliciTI has no low power delay function, so we have to use ours
extern void Timer0_A4_Delay(u16 ticks);


// *************************************************************************************************
// Global Variable section
static linkID_t sLinkID1;
static uint8_t s_syncDone = 0;

static volatile uint32_t index = 0;
void olimex_delay(unsigned long delay);
// *************************************************************************************************
// @fn          simpliciti_link
// @brief       Init hardware and try to link to access point.
// @param       none
// @return      unsigned char		0 = Could not link, timeout or external cancel.
//									1 = Linked successful.
// *************************************************************************************************
unsigned char simpliciti_link(void)
{
    uint8_t timeout;
    addr_t lAddr;
    uint8_t i;
    uint8_t pwr;

    // Configure timer
    BSP_InitBoard();

    // Change network address to value set in calling function
    for (i = 0; i < NET_ADDR_SIZE; i++)
    {
        lAddr.addr[i] = simpliciti_ed_address[i];
    }
    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);

    // Set flag
    simpliciti_flag = SIMPLICITI_STATUS_LINKING;

    /* Keep trying to join (a side effect of successful initialization) until
     * successful. Toggle LEDS to indicate that joining has not occurred.
     */
    timeout = 0;
    while (SMPL_SUCCESS != SMPL_Init(0))
    {
        NWK_DELAY(1000);

#ifdef USE_WATCHDOG
        // Service watchdog
        WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK + WDTCNTCL;
#endif

        // Stop connecting after defined numbers of seconds (15)
        if (timeout++ > TIMEOUT)
        {
            // Clean up SimpliciTI stack to enable restarting
            sInit_done = 0;
            simpliciti_flag = SIMPLICITI_STATUS_ERROR;
            return (0);
        }

        // Break when flag bit SIMPLICITI_TRIGGER_STOP is set
        if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP))
        {
            // Clean up SimpliciTI stack to enable restarting
            sInit_done = 0;
            return (0);
        }
    }

    // Set output power to +3.3dmB
    pwr = IOCTL_LEVEL_2;
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SETPWR, &pwr);

    /* Unconditional link to AP which is listening due to successful join. */
    timeout = 0;
    while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
    {
        NWK_DELAY(1000);

#ifdef USE_WATCHDOG
        // Service watchdog
        WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK + WDTCNTCL;
#endif

        // Stop linking after timeout
        if (timeout++ > TIMEOUT)
        {
            // Clean up SimpliciTI stack to enable restarting
            sInit_done = 0;
            simpliciti_flag = SIMPLICITI_STATUS_ERROR;
            return (0);
        }

        // Exit when flag bit SIMPLICITI_TRIGGER_STOP is set
        if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP))
        {
            // Clean up SimpliciTI stack to enable restarting
            sInit_done = 0;
            return (0);
        }
    }
    simpliciti_flag = SIMPLICITI_STATUS_LINKED;

    return (1);
}

// *************************************************************************************************
// @fn          simpliciti_main_tx_only
// @brief       Get data through callback. Transfer data when external trigger is set.
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_main_tx_only(void)
{
	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

	while (1)
	{
		smplStatus_t returnCode = SMPL_SendOpt(sLinkID1, simpliciti_data + 1, simpliciti_data[0], SMPL_TXOPTION_ACKREQ);

		if (returnCode != SMPL_SUCCESS)
		{
			continue;
		}

		break;
	}

	SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXIDLE, 0);

    clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_DATA);
}

// *************************************************************************************************
// @fn          simpliciti_main_sync
// @brief       Send ready-to-receive packets in regular intervals. Listen shortly for host reply.
//				Decode received host command and trigger action.
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_main_sync(void)
{
    uint8_t len;
    uint8_t ed_data[2];

    if (s_syncDone)
    {
    	return;
    }

        // Send 2 byte long ready-to-receive packet to stimulate host reply
        ed_data[0] = SYNC_ED_TYPE_R2R;
        ed_data[1] = 0xCB;

        SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

        smplStatus_t status = SMPL_NO_ACK;
        while (status == SMPL_NO_ACK)
        {
        	status = SMPL_SendOpt(sLinkID1, ed_data, 2, SMPL_TXOPTION_ACKREQ);
        }

        NWK_DELAY(10);
        uint8_t retries = 10;
        while (retries--)
        {
        	if (SMPL_Receive(sLinkID1, simpliciti_data, &len) != SMPL_SUCCESS)
        	{
        		NWK_DELAY(10);
        		continue;
        	}

        	if (simpliciti_data[0] == SYNC_AP_CMD_SET_TIME_T && len == 5)
        	{
        			uint32_t timeT = 0;
        			timeT |= simpliciti_data[1];
        			timeT |= ((uint32_t)simpliciti_data[2] << 8);
        			timeT |= ((uint32_t)simpliciti_data[3] << 16);
        			timeT |= ((uint32_t)simpliciti_data[4] << 24);

        			timestampInit(timeT);

        			s_syncDone = 1;

        			break;
        	}
        }

        SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXIDLE, 0);

#ifdef USE_WATCHDOG
        // Service watchdog
        WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK + WDTCNTCL;
#endif
}

void sendShmData()
{
	open_radio();

	uint32_t index = 1;

	if (simpliciti_link())
	{
		// Get radio ready. Wakes up in IDLE state.
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);

		simpliciti_main_sync();

		setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_DATA);

		while (1)
		{
			battery_measurement();
			temperature_measurement(FILTER_OFF);

			u16 voltage = sBatt.voltage;
			s16 degrees = sTemp.degrees;

			memcpy(simpliciti_data + 1, timestampAsBuffer(), 6);
			memcpy(simpliciti_data + 7, &index, 4);

			simpliciti_data[11] = 'B';
			memcpy(simpliciti_data + 12, (uint8_t*)&voltage, 2);
			simpliciti_data[14] = 'T';
			memcpy(simpliciti_data + 15, (uint8_t*)&degrees, 2);
			simpliciti_data[0] = 16;

			simpliciti_main_tx_only();

			index++;

			BSP_TURN_OFF_LED1();
			Timer0_A4_Delay(CONV_MS_TO_TICKS(1000));
			BSP_TURN_ON_LED1();
		}

		clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_DATA);
	}

    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
    sInit_done = 0;

    close_radio();
}

