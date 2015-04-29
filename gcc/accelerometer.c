#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

#include "accelerometer.h"


extern uint16_t acceletometer_data[];

void accel_setup(void){
    
    setAcceltoActive();
}

void setAcceltoActive() {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x2A);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterDataPut(I2C1_BASE, 0xC1);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
}

void setAcceltoStandby() {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x2A);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterDataPut(I2C1_BASE, 0xC0);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
}

void readAccelData() {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x01);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, true);
    //read x
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[0] = ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[0] = acceletometer_data[0] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[0] = acceletometer_data[0] | ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[0] = acceletometer_data[0] >> 4;
    //read y
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[1] = ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[1] = acceletometer_data[1] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[1] = acceletometer_data[1] | ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[1] = acceletometer_data[1] >> 4;
    //read z
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[2] = ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[2] = acceletometer_data[2] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    acceletometer_data[2] = acceletometer_data[2] | ROM_I2CMasterDataGet(I2C1_BASE);
    acceletometer_data[2] = acceletometer_data[2] >> 4;
}

void calcTiltAnglesFromAccelData() {
    //converting tilt angle on X-Axis
    acceletometer_data[0] = (acceletometer_data[0] >= 0) ? asinf( ((acceletometer_data[0]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*asinf( (((-1)*acceletometer_data[0]) / 2047.0)*2.0 )* 180 / M_PI;
    //converting tilt angle on Y-Axis
    acceletometer_data[1] = (acceletometer_data[1] >= 0) ? asinf( ((acceletometer_data[1]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*asinf( (((-1)*acceletometer_data[1]) / 2047.0)*2.0 )* 180 / M_PI;
    //converting tilt angle on Z-Axis
    acceletometer_data[2] = (acceletometer_data[2] >= 0) ? acosf( ((acceletometer_data[2]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*acosf( (((-1)*acceletometer_data[2]) / 2047.0)*2.0 )* 180 / M_PI;
}

