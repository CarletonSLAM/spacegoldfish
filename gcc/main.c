#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
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

void initI2C1(void);
void I2CReceive(uint32_t slaveAddress, char retData[]);
void resetCMD(void);
char promRead();

int main(void){

    char temp[3];
    
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    initI2C1();

    ///////////UART SHIT
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    ///////////////////////

    resetCMD();
    I2CReceive(0xEE, temp);
    temp[1] = promRead();
    ROM_UARTCharPutNonBlocking(UART0_BASE, temp[0]);
    ROM_UARTCharPutNonBlocking(UART0_BASE, temp[1]);
    ROM_UARTCharPutNonBlocking(UART0_BASE, temp[2]);


}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        while (ROM_UARTBusy(UART0_BASE));
    }
}

void initI2C1(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), false);
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;

}

void resetCMD(void) {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0xEE, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x1E);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    return;
}

char promRead() {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0xEE, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0xA6);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0xEE, true);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    return ROM_I2CMasterDataGet(I2C1_BASE);
}

void I2CReceive(uint32_t slaveAddress, char retData[]) {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0xEE, false);
    ROM_I2CMasterDataPut(I2C1_BASE, 0x42);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterDataPut(I2C1_BASE, 0x00);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, 0xEE, true);

    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    retData[0] = ROM_I2CMasterDataGet(I2C1_BASE);

    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    retData[1] = ROM_I2CMasterDataGet(I2C1_BASE);

    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    retData[2] = ROM_I2CMasterDataGet(I2C1_BASE);
}
