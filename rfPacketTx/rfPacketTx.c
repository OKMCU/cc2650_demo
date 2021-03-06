/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Board Header files */
#include "Board.h"

#include "smartrf_settings/smartrf_settings.h"

/* Pin driver handles */
static PIN_Handle buttonPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/***** Defines *****/
#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   2

#define LED_TASK_STACK_SIZE 1024

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      30
#define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */



/***** Prototypes *****/
static void txTaskFunction(UArg arg0, UArg arg1);
static void ledTaskFunction(UArg arg0, UArg arg1);


/***** Variable declarations *****/
static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static Task_Params ledTaskParams;
Task_Struct ledTask;
static uint8_t ledTaskStack[LED_TASK_STACK_SIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;

uint32_t time;
static uint8_t packet[PAYLOAD_LENGTH];

static uint8_t pktData = 0x00;      /* Packet Data, 
                                       button0 pressed   = 0xAA, led0   blink on both transmitter and receiver
                                       button1 pressed   = 0x55, led1   blink on both transmitter and receiver
                                       no button pressed = 0x00, led0&1 blink on both transmitter and receiver
                                    */
/***** Function definitions *****/
void TxTask_init(void)
{
    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, NULL);
}

void LedTask_init(uint16_t update_interval)
{
    Task_Params_init(&ledTaskParams);
    ledTaskParams.stackSize = LED_TASK_STACK_SIZE;
    ledTaskParams.stack = &ledTaskStack;
    ledTaskParams.arg0 = update_interval;

    Task_construct(&ledTask, ledTaskFunction, &ledTaskParams, NULL);
}

static void txTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t time;
    uint8_t tx_byte;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    time = RF_getCurrentTime();
    while(1)
    {
        tx_byte = pktData;
        /* Create packet with incrementing sequence number and random payload */
        uint8_t i;
        for (i = 0; i < PAYLOAD_LENGTH; i++)
        {
            packet[i] = tx_byte;
        }

        /* Set absolute TX time to utilize automatic power management */
        time += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = time;

        /* Send packet */
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
        if (!(result & RF_EventLastCmdDone))
        {
            /* Error */
            while(1);
        }
    }
}

/*
 *  ======== ledTaskFunction ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
Void ledTaskFunction(UArg arg0, UArg arg1)
{
    PWM_Handle pwm1, pwm2;
    PWM_Params params;
    uint16_t   pwmPeriod = 5000;      // Period and duty in microseconds
    uint16_t   duty = 0;
    uint16_t   dutyInc = 5;
    int8_t     dir = 1;

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);
    GPIO_write(Board_LED1, Board_LED_ON);

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);
    pwm2 = PWM_open(Board_PWM1, &params);

    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }

    if (pwm2 == NULL) {
        System_abort("Board_PWM1 did not open");
    }
    PWM_start(pwm1);
    PWM_start(pwm2);
    /* Loop forever incrementing the PWM duty */
    while (1) {

        if(pktData == 0x00)
        {
            PWM_setDuty(pwm1, duty);
            PWM_setDuty(pwm2, duty);
        }
        else if(pktData == 0xAA)
        {
            PWM_setDuty(pwm1, duty);
            PWM_setDuty(pwm2, 0);
        }
        else if(pktData == 0x55)
        {
            PWM_setDuty(pwm1, 0);
            PWM_setDuty(pwm2, duty);
        }
        else
        {
            PWM_setDuty(pwm1, 0);
            PWM_setDuty(pwm2, 0);
        }
        
        if(dir > 0)
        {
            if(duty + dutyInc >= pwmPeriod)
            {
                dir = ~dir;
            }
            else
            {
                duty += dutyInc;
            }
        }
        else if(dir < 0)
        {
            if(duty <= dutyInc)
            {
                dir = ~dir;
            }
            else
            {
                duty -= dutyInc;
            }
        }

        Task_sleep((UInt) arg0);
    }
}

/*
 *  ======== buttonCallbackFxn ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 *  If Board_LED3 and Board_LED4 are defined, then we'll add them to the PIN
 *  callback function.
 */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    //uint32_t currVal = 0;

    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
        /* Toggle LED based on the button pressed */
        switch (pinId) {
            case Board_BUTTON0:
                //currVal =  PIN_getOutputValue(Board_LED0);
                //PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
                pktData = 0xAA;
                break;

            case Board_BUTTON1:
                //currVal =  PIN_getOutputValue(Board_LED1);
                //PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);
                pktData = 0x55;
                break;

            default:
                /* Do nothing */
                break;
        }
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initPWM();

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    /* Initialize task */
    TxTask_init();
    LedTask_init(50);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
