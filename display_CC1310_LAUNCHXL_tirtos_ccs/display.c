/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

/*
 *  ======== display.c ========
 */
#include <ti/drivers/Power.h>

#include <ti/devices/cc13x0/driverlib/aon_batmon.h>
#include <ti/devices/cc13x0/driverlib/aux_adc.h>
#include <ti/devices/cc13x0/driverlib/chipinfo.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#include <stdint.h>
#include <unistd.h>

/* TI-Drivers Header files */
#include <ti/drivers/GPIO.h>

/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>

/* Board Header files */
#include "Board.h"

/* Example GrLib image */
#include "splash_image.h"

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    unsigned int ledPinValue;
    unsigned int loopCount = 0;

    GPIO_init();
    Display_init();

    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

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
    Display_Handle hLcd = Display_open(Display_Type_LCD, &params);
    Display_Handle hSerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    if (hSerial) {
        Display_printf(hSerial, 0, 0, "Hello Serial!");
    }
    else
    {
        /* Print unavail message on LCD. Note it's not necessary to
         * check for NULL handles with Display APIs, so just assume hLcd works.
         */
        Display_printf(hLcd, 4, 0, "Serial display");
        Display_printf(hLcd, 5, 0, "not present");
        sleep(1);
    }

    /* Check if the selected Display type was found and successfully opened */
    if (hLcd) {
        Display_printf(hLcd, 5, 3, "Hello LCD!");

        /* Wait a while so text can be viewed. */
        sleep(3);

        /*
         * Use the GrLib extension to get the GraphicsLib context object of the
         * LCD, if it is supported by the display type.
         */
        Graphics_Context *context = DisplayExt_getGraphicsContext(hLcd);

        /* It's possible that no compatible display is available. */
        if (context) {
            /* Draw splash */
            Graphics_drawImage(context, &splashImage, 0, 0);
            Graphics_flushBuffer(context);
        }
        else {
            /* Not all displays have a GraphicsLib back-end */
            Display_printf(hLcd, 0, 0, "Display driver");
            Display_printf(hLcd, 1, 0, "is not");
            Display_printf(hLcd, 2, 0, "GrLib capable!");
        }

        /* Wait for a bit, then clear */
        sleep(3);
        Display_clear(hLcd);
    }
    else
    {
        Display_printf(hSerial, 1, 0, "LCD display not present");
        sleep(1);
    }

    char *serialLedOn = "On";
    char *serialLedOff = "Off";

    /* If serial display can handle ANSI colors, use colored strings instead.
     *
     * You configure DisplayUart to use the ANSI variant by choosing what
     * function pointer list it should use. Ex:
     *
     * const Display_Config Display_config[] = {
     *   {
     *      .fxnTablePtr = &DisplayUartAnsi_fxnTable, // Alt: DisplayUartMin_
     *      ...
     */
    if (Display_getType(hSerial) & Display_Type_ANSI)
    {
        serialLedOn = ANSI_COLOR(FG_GREEN, ATTR_BOLD) "On" ANSI_COLOR(ATTR_RESET);
        serialLedOff = ANSI_COLOR(FG_RED, ATTR_UNDERLINE) "Off" ANSI_COLOR(ATTR_RESET);
    }

    AONBatMonEnable();
    while (1)
    {
        if(AONBatMonNewTempMeasureReady())
        {
            Display_printf(hSerial, 0, 5, "temperature turn to = %d",AONBatMonTemperatureGetDegC());
        }
        else
        {
            Display_printf(hSerial, 0, 3,"temperature: %d is the latest",AONBatMonTemperatureGetDegC());
        }
        sleep(3);
    }
}
