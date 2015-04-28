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


#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif

void initI2C1(void);
void setAcceltoActive(void);
void readAccelData(int16_t dataBuf[]);
void setAcceltoStandby(void);
void calcTiltAnglesFromAccelData(int16_t dataBuf[]);
void uartInit_DEBUG(void);
void uartTransmitTiltAngles_DEBUG(int16_t dataBuf[]);

int main(void){

    int16_t accelerometerData[3];
    ROM_FPULazyStackingEnable();
    ROM_FPUEnable();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    
    initI2C1();
    uartInit_DEBUG();
    setAcceltoActive();
    readAccelData(accelerometerData);
    calcTiltAnglesFromAccelData(accelerometerData);
    uartTransmitTiltAngles_DEBUG(accelerometerData);

}

void uartInit_DEBUG() {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void uartTransmitTiltAngles_DEBUG(int16_t dataBuf[]) {
    char trasmitBuffer[3];
    for (int i = 0; i<3; i++) {

        if (i == 0) {
            ROM_UARTCharPutNonBlocking(UART0_BASE, 'X');
        }
        else if (i == 1) {
            ROM_UARTCharPutNonBlocking(UART0_BASE, 'Y');
        }
        else if (i == 2) {
            ROM_UARTCharPutNonBlocking(UART0_BASE, 'Z');
        }

        if (dataBuf[i] < 0) {
            dataBuf[i] *= -1;
            ROM_UARTCharPutNonBlocking(UART0_BASE, '-');
        }
        /*
        trasmitBuffer[0] = dataBuf[i]/100;
        trasmitBuffer[1] = dataBuf[i]/10;
        trasmitBuffer[2] = dataBuf[i]%10;
        */
        trasmitBuffer[0] = dataBuf[i]/10;
        trasmitBuffer[1] = dataBuf[i]%10;
        ROM_UARTCharPutNonBlocking(UART0_BASE, trasmitBuffer[0] + '0');
        ROM_UARTCharPutNonBlocking(UART0_BASE, trasmitBuffer[1] + '0');
        ROM_UARTCharPutNonBlocking(UART0_BASE, ' ');
    }
}


void initI2C1(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), false);
    
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
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

void readAccelData(int16_t dataBuf[]) {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x01);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0x1D, true);
    //read x
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[0] = ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[0] = dataBuf[0] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[0] = dataBuf[0] | ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[0] = dataBuf[0] >> 4;
    //read y
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[1] = ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[1] = dataBuf[1] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[1] = dataBuf[1] | ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[1] = dataBuf[1] >> 4;
    //read z
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[2] = ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[2] = dataBuf[2] << 8;
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    dataBuf[2] = dataBuf[2] | ROM_I2CMasterDataGet(I2C1_BASE);
    dataBuf[2] = dataBuf[2] >> 4;
}

void calcTiltAnglesFromAccelData(int16_t dataBuf[]) {
    //converting tilt angle on X-Axis
    dataBuf[0] = (dataBuf[0] >= 0) ? asinf( ((dataBuf[0]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*asinf( (((-1)*dataBuf[0]) / 2047.0)*2.0 )* 180 / M_PI;
    //converting tilt angle on Y-Axis
    dataBuf[1] = (dataBuf[1] >= 0) ? asinf( ((dataBuf[1]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*asinf( (((-1)*dataBuf[1]) / 2047.0)*2.0 )* 180 / M_PI;
    //converting tilt angle on Z-Axis
    dataBuf[2] = (dataBuf[2] >= 0) ? acosf( ((dataBuf[2]) / 2047.0)*2.0 )* 180 / M_PI 
                                   : (-1)*acosf( (((-1)*dataBuf[2]) / 2047.0)*2.0 )* 180 / M_PI;
}

