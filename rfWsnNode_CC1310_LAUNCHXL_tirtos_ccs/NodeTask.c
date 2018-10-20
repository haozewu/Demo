/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */ 
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "SceAdc.h"
#include "NodeTask.h"
#include "NodeRadioTask.h"

/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_NEW_ADC_VALUE    (uint32_t)(1 << 0)
#define NODE_EVENT_UPDATE_LCD       (uint32_t)(1 << 1)

/* A change mask of 0xFF0 means that changes in the lower 4 bits does not trigger a wakeup. */
#define NODE_ADCTASK_CHANGE_MASK                    0xFF0

/* Minimum slow Report interval is 50s (in units of samplingTime)*/
#define NODE_ADCTASK_REPORTINTERVAL_SLOW                50
/* Minimum fast Report interval is 1s (in units of samplingTime) for 30s*/
#define NODE_ADCTASK_REPORTINTERVAL_FAST                5
#define NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS   30000


#define NUM_EDDYSTONE_URLS      5

/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* Not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* Not static so you can see in ROV */
static Event_Handle nodeEventHandle;
static uint16_t latestAdcValue;

/* Clock for the fast report timeout */
Clock_Struct fastReportTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle fastReportTimeoutClockHandle;

/* Pin driver handle */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;
static PIN_State buttonPinState;
static PIN_State ledPinState;

/* Display driver handles */
static Display_Handle hDisplaySerial;
/* Enable the 3.3V power domain used by the LCD */
PIN_Config pinTable[] = {
#if !defined Board_CC1350STK
    NODE_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

static uint8_t nodeAddress = 0;

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
static void updateLcd(void);
static void fastReportTimeoutCallback(UArg arg0);
static void adcCallback(uint16_t adcValue);
static void buttonCallback(PIN_Handle handle, PIN_Id pinId);


/***** Function definitions *****/
void NodeTask_init(void)
{

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    /* Create clock object which is used for fast report timeout */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);

    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&fastReportTimeoutClock, fastReportTimeoutCallback, 1, &clkParams);
    fastReportTimeoutClockHandle = Clock_handle(&fastReportTimeoutClock);

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open both UART and LCD types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySerial)
    {
        Display_printf(hDisplaySerial, 0, 0, "Waiting for SCE ADC reading...");
    }
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Start the SCE ADC task with 1s sample period and reacting to change in ADC value. */
    SceAdc_init(0x00010000, NODE_ADCTASK_REPORTINTERVAL_FAST, NODE_ADCTASK_CHANGE_MASK);
    SceAdc_registerAdcCallback(adcCallback);
    SceAdc_start();

    /* setup timeout for fast report timeout */
    Clock_setTimeout(fastReportTimeoutClockHandle,
            NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS * 1000 / Clock_tickPeriod);

    /* Start fast report and timeout */
    Clock_start(fastReportTimeoutClockHandle);


    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if (!buttonPinHandle)
    {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallback) != 0)
    {
        System_abort("Error registering button callback function");
    }

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If new ADC value, send this data */
        if (events & NODE_EVENT_NEW_ADC_VALUE) {
            /* Toggle activity LED */
#if !defined Board_CC1350STK
            PIN_setOutputValue(ledPinHandle, NODE_ACTIVITY_LED,!PIN_getOutputValue(NODE_ACTIVITY_LED));
#endif

            /* Send ADC value to concentrator */
            NodeRadioTask_sendAdcData(latestAdcValue);

            /* Update display */
            updateLcd();
        }
        /* If new ADC value, send this data */
        if (events & NODE_EVENT_UPDATE_LCD) {
            /* update display */
            updateLcd();
        }
    }
}

static void updateLcd(void)
{

    /* get node address if not already done */
    if (nodeAddress == 0)
    {
        nodeAddress = nodeRadioTask_getNodeAddr();
    }
    /* Print to UART clear screen, put cuser to beggining of terminal and print the header */
    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HNode ID: 0x%02x", nodeAddress);
    Display_printf(hDisplaySerial, 0, 0, "Node ADC Reading: %04d", latestAdcValue);
}
static void adcCallback(uint16_t adcValue)
{
    /* Save latest value */
    latestAdcValue = adcValue;

    /* Post event */
    Event_post(nodeEventHandle, NODE_EVENT_NEW_ADC_VALUE);
}

/*
 *  ======== buttonCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */
static void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);

    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
    {
        //start fast report and timeout
        SceAdc_setReportInterval(NODE_ADCTASK_REPORTINTERVAL_FAST, NODE_ADCTASK_CHANGE_MASK);
        Clock_start(fastReportTimeoutClockHandle);
    }
}

static void fastReportTimeoutCallback(UArg arg0)
{
    //stop fast report
    SceAdc_setReportInterval(NODE_ADCTASK_REPORTINTERVAL_SLOW, NODE_ADCTASK_CHANGE_MASK);
}
