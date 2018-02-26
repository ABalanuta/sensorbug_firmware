/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/*
 *  ======== sensorTestService.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#if defined(CC2650DK_7ID) || defined(CC1310DK_7XD)
#include <ti/drivers/PIN.h>
#endif

/* Example/Board Header files */
#include "Config/Board_LoRaBUG.h"
#include <stdio.h>
#include <stdlib.h>
#include "io.h"
#include "Services/bmeService.h"
#include "Services/bmxService.h"
#include "Services/grideyeService.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define SENSORTEST_TASK_PRIORITY                     5

#define SENSORTEST_TASK_STACK_SIZE                   2000


#define LED_PIN_RX      Board_GLED
#define LED_PIN_TX      Board_RLED

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)


/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
Task_Struct sensorTestTask;
Char sensorTestTaskStack[SENSORTEST_TASK_STACK_SIZE];

/*
 *  ======== taskFxn1 ========
 *  Open a ADC handle and get a array of sampling results after
 *  calling several conversions.
 */
Void sensorTest_taskFxn(UArg arg0, UArg arg1)
{
    setLed(Board_GLED, false);
    setLed(Board_RLED, false);

    //Test the GridEye chip. If anything fails, assert red LED
    grideye_init();
    while(1) {
        //Try and get data from BMX. If anything fails, will assert red LED
        uint8_t bmxData[20];
        getBmxData(bmxData);

        float accelx, accely, accelz, gyrx, gyry, gyrz, magx, magy, magz;

        accelz = ((float)((((uint32_t)bmxData[19]) << 8) | (uint32_t)bmxData[18])) * (2.0 / 32767);
        accely = ((float)((((uint32_t)bmxData[17]) << 8) | (uint32_t)bmxData[16])) * (2.0 / 32767);
        accelx = ((float)((((uint32_t)bmxData[15]) << 8) | (uint32_t)bmxData[14])) * (2.0 / 32767);

        gyrz = ((float)((((uint32_t)bmxData[13]) << 8) | bmxData[12])) * (2000.0 / 32767);
        gyry = ((float)((((uint32_t)bmxData[11]) << 8) | bmxData[10])) * (2000.0 / 32767);
        gyrx = ((float)((((uint32_t)bmxData[9]) << 8) | bmxData[8])) * (2000.0 / 32767);

        magz = ((float)((((uint32_t)bmxData[5]) << 8) | bmxData[4])) * (2500.0 / 32767);
        magy = ((float)((((uint32_t)bmxData[3]) << 8) | bmxData[2])) * (1300.0 / 32767);
        magx = ((float)((((uint32_t)bmxData[1]) << 8) | bmxData[0])) * (1300.0 / 32767);

        uartputs("bmx160 readings:\r\n");
        uartprintf("Accel X: %f Y: %f Z: %f\r\n", accelx, accely, accelz);
        uartprintf("Gyr X: %f Y: %f Z: %f\r\n", gyrx, gyry, gyrz);
        uartprintf("Mag X: %f Y: %f, Z: %f\r\n", magx, magy, magz);
        uartputs("\r\n");

        float temp, pres, hum;
        //Get BME readings
        getBMEData(&temp, &pres, &hum);

        uartputs("bme680 readings: \r\n");
        uartprintf("Temp: %f Pres: %f Hum: %f\r\n", temp, pres, hum);

        //The other sensors

        setPin(Board_HDR_HDIO1, true);
        ADC_Handle   adc, adc1;
        ADC_Params   params, params1;
        int_fast16_t res, res1;

        ADC_Params_init(&params);
        adc = ADC_open(2, &params);

        if (adc == NULL) {
           DELAY_MS(100);
           System_abort("ADC err\n");
        }


        uint16_t adcValue0, adcValue1;
        uint16_t minV, maxV;

        minV = 0xFFFF;
        maxV = 0;

        uint32_t currTicks, startTicks;

        startTicks = Clock_getTicks();
        currTicks = startTicks;

        while((currTicks - startTicks) < 5000) {
           currTicks = Clock_getTicks();
           res = ADC_convert(adc, &adcValue0);
           if (res == ADC_STATUS_SUCCESS) {
               if(maxV < adcValue0)
                   maxV = adcValue0;
               if(minV > adcValue0)
                   minV = adcValue0;
           }
           else {
               uartprintf("ADConverr\r\n");
           }

        }
        ADC_close(adc);

        ADC_Params_init(&params1);
        adc = ADC_open(0, &params1);

        if (adc == NULL) {
           DELAY_MS(100);
           System_abort("ADC err2\n");
        }


        startTicks = Clock_getTicks();
        currTicks = startTicks;
        uint32_t lightAvg = 0, count = 0;


        while((currTicks - startTicks) < 5000) {
           currTicks = Clock_getTicks();
           res = ADC_convert(adc, &adcValue1);
           if (res == ADC_STATUS_SUCCESS) {
               lightAvg += adcValue1;
               count++;
           }
           else {
               uartprintf("ADConverr2\r\n");
           }

        }
        ADC_close(adc);
        lightAvg = lightAvg/count;

        setPin(Board_HDR_HDIO1, false);

        uint32_t pir_status = 0;

        //Get PIR status
        startTicks = Clock_getTicks();
        currTicks = startTicks;
        while((currTicks - startTicks) < 5000){
           currTicks = Clock_getTicks();
           pir_status = getPin(Board_HDR_ADIO6);
           if(pir_status == 1)
               break;
        }

    //    message.mic = maxV - minV;
    //    message.pir_status = pir_status;
    //    message.light = lightAvg;

        uartprintf("The microphone reading: %d\r\n", maxV - minV);
        uartprintf("The PIR status: %d\r\n", pir_status);
        uartprintf("The light sensor reading: %d\r\n", lightAvg);

        DELAY_MS(2000);

    }
    //Assert the green LED to signal completion of tests
//    setLed(Board_GLED, true);

}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      sensorTestService_createTask
 *
 * @brief   Task creation function for the light sensor application.
 *
 * @param   None.
 *
 * @return  None.
 */

void sensorTestService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sensorTestTaskStack;
  taskParams.stackSize = SENSORTEST_TASK_STACK_SIZE;
  //taskParams.priority = LIGHT_TASK_PRIORITY;

  Task_construct(&sensorTestTask, sensorTest_taskFxn, &taskParams, NULL);
}


