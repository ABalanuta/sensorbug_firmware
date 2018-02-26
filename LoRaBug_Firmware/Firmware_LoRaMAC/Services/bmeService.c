/******************************************************************************

  @file  bmeService.c

 @brief This file contains a simple I2C transaction with the bme680
         Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_BMESERVICE_C_
#define SERVICES_BMESERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <io.h>

#include <Services/bme680.h>
#include <Services/bme680_selftest.h>


/* Board Header files */
#include "Config/Board_LoRaBUG.h"

//#include "loraCommon.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define BME_TASK_PRIORITY                     5

#define BME_TASK_STACK_SIZE                   1200


#define LED_PIN_RX      Board_GLED
#define LED_PIN_TX      Board_RLED

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
Task_Struct bmeTask;
Char bmeTaskStack[BME_TASK_STACK_SIZE];


static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
//    uartputs("i2c read called \r\n");
    int8_t rslt = 0; //0 for success, non-zero for failure
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans1;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;


    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        rslt = -1;
    }
    else {
        i2cTrans1.slaveAddress = dev_id;
        i2cTrans1.writeBuf = &reg_addr;
        i2cTrans1.writeCount = 1;
        i2cTrans1.readBuf = reg_data;
        i2cTrans1.readCount = len;


//        uartprintf("The provided slave address is : %x\r\n", dev_id);
//        uartprintf("The register address being written is : %x \r\n", reg_addr);

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans1);
        setLed(Board_RLED, false);
        setLed(Board_GLED, false);
        if(!status) {
            setLed(Board_RLED, true);
            rslt = -1;
        }
        else {
//            setLed(Board_GLED, true);
        }

        I2C_close(handle);
    }
//    uartprintf("Done reading with result : %d \r\n", rslt);
    return rslt;
}



static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
//    uartputs("i2c write called \r\n");
    int8_t rslt = 0; //0 for success, non-zero for failure
    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction i2cTrans1;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;


//    uartprintf("The provided slave address is : %x\r\n", dev_id);
//    uartprintf("The register address being written is : %x \r\n", reg_addr);

    handle = I2C_open(Board_I2C, &params);
    if(!handle) {
        rslt = -1;
        uartputs("Opening failed \r\n");
    }
    else {
        uint8_t *txBuf = malloc((len + 1) * sizeof(uint8_t));
        i2cTrans1.slaveAddress = dev_id;
        i2cTrans1.writeBuf = txBuf;
        i2cTrans1.writeCount = len + 1;
        i2cTrans1.readBuf = NULL;
        i2cTrans1.readCount = 0;
        txBuf[0] = reg_addr;
        for(int j = 0; j < len; j++)
        {
            txBuf[1 + j] = reg_data[j];
        }

        bool status = false;
        status = I2C_transfer(handle, &i2cTrans1);
        setLed(Board_RLED, false);
        setLed(Board_GLED, false);
        if(!status) {
            uartputs("The transfer failed \r\n");
            setLed(Board_RLED, true);
        }
        else {
//            uartputs("The transfer was successful \r\n");
//            setLed(Board_GLED, true);
        }
        free(txBuf);
        I2C_close(handle);
    }
//    uartprintf("Done writing with result : %d \r\n", rslt);
    return rslt;
}

void user_delay_ms(uint32_t period)
{
    DELAY_MS(period);
}

void getBMEData(float *temp, float *pres, float *hum) {
    struct bme680_dev gas_sensor;

        //Configuring I2C communication interface
        gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
        gas_sensor.intf = BME680_I2C_INTF;
        gas_sensor.read = user_i2c_read;
        gas_sensor.write = user_i2c_write;
        gas_sensor.delay_ms = user_delay_ms;


        //Initializing object to default values
        int8_t rslt = BME680_OK;
        rslt = bme680_init(&gas_sensor);
        if(rslt != BME680_OK)
            uartprintf("Init fail : %d \r\n", rslt);

        uint8_t set_required_settings;

        /* Set the temperature, pressure and humidity settings */
        gas_sensor.tph_sett.os_hum = BME680_OS_2X;
        gas_sensor.tph_sett.os_pres = BME680_OS_4X;
        gas_sensor.tph_sett.os_temp = BME680_OS_8X;
        gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

        /* Set the remaining gas sensor settings and link the heating profile */
        gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
        /* Create a ramp heat waveform in 3 steps */
        gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
        gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

        /* Select the power mode */
        /* Must be set before writing the sensor configuration */
        gas_sensor.power_mode = BME680_FORCED_MODE;

        /* Set the required sensor settings needed */
        set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
            | BME680_GAS_SENSOR_SEL;

        /* Set the desired sensor configuration */
        rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
        if(rslt != 0) {
            setLed(Board_RLED, true);
            uartprintf("sensor fail : %d\r\n", rslt);
        }


        /* Set the power mode */
        rslt = bme680_set_sensor_mode(&gas_sensor);
        if(rslt != 0) {
            setLed(Board_RLED, true);
            uartprintf("power mode fail : %d \r\n", rslt);
        }


        /* Get the total measurement duration so as to sleep or wait till the
         * measurement is complete */
        uint16_t meas_period;
        bme680_get_profile_dur(&meas_period, &gas_sensor);
        DELAY_MS(meas_period); /* Delay till the measurement is ready */

        struct bme680_field_data data;

        rslt = bme680_get_sensor_data(&data, &gas_sensor);

        uartprintf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            uartprintf(", G: %d ohms", data.gas_resistance);

        uartprintf("\r\n");

        *temp = data.temperature / 100.0f;
        *pres = data.pressure / 100.0f;
        *hum = data.humidity / 1000.0f;
        uartprintf("Done reading BME dataa\r\n");
}

void testBMEChip() {
    struct bme680_dev gas_sensor;

    //Configuring I2C communication interface
    gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = user_i2c_read;
    gas_sensor.write = user_i2c_write;
    gas_sensor.delay_ms = user_delay_ms;


    //Initializing object to default values
    int8_t rslt = BME680_OK;
    rslt = bme680_init(&gas_sensor);
    if(rslt != BME680_OK)
        uartprintf("Init fail : %d \r\n", rslt);

    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
    if(rslt != 0) {
        setLed(Board_RLED, true);
        uartprintf("sensor fail : %d\r\n", rslt);
    }


    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
    if(rslt != 0) {
        setLed(Board_RLED, true);
        uartprintf("power mode fail : %d \r\n", rslt);
    }


    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);
    DELAY_MS(meas_period); /* Delay till the measurement is ready */

    struct bme680_field_data data;

    rslt = bme680_get_sensor_data(&data, &gas_sensor);

    uartprintf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
        data.pressure / 100.0f, data.humidity / 1000.0f );
    /* Avoid using measurements from an unstable heating setup */
    if(data.status & BME680_GASM_VALID_MSK)
        uartprintf(", G: %d ohms", data.gas_resistance);

    uartprintf("\r\n");

}
/*********************************************************************
 * @fn      bmx_taskFxn
 * @return  None.
 */
static void bme_taskFxn (UArg a0, UArg a1)
{
    while(1){
        uint8_t txBuf[2];
        uint8_t rxBuf[2];
        txBuf[0] = 0xFF;
        user_i2c_write((uint8_t)0x77, (uint8_t)0xD0, txBuf, 1);
        user_i2c_read((uint8_t)0x77, (uint8_t)0x1F, rxBuf, 1);
        DELAY_MS(3000);
    }

}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bmeService_createTask
 *
 * @brief   Task creation function for the bme680 application.
 *
 * @param   None.
 *
 * @return  None.
 */
void bmeService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = bmeTaskStack;
  taskParams.stackSize = BME_TASK_STACK_SIZE;
  //taskParams.priority = BME_TASK_PRIORITY;

  Task_construct(&bmeTask, bme_taskFxn, &taskParams, NULL);
}



#endif /* SERVICES_BMESERVICE_C_ */
