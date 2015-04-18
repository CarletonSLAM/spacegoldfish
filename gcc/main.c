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
uint32_t I2CReceive(uint32_t slaveAddress, uint8_t reg);

int main(void){
    uint32_t temp;
    uint32_t i;
    
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    initI2C1();

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    while (1) {
        for (i = 0; i<1000;i++){        temp = I2CReceive(0x0E, 0x01);
        UARTSend((uint8_t*) temp, 1);
        ROM_UARTCharPutNonBlocking(UART0_BASE, (char)temp);
    }}
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
   ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), false);
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}

uint32_t I2CReceive(uint32_t slaveAddress, uint8_t reg) {
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, slaveAddress, false);
    ROM_I2CMasterDataPut(I2C1_BASE, reg);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, slaveAddress, true);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    while(ROM_I2CMasterBusy(I2C1_BASE));
//  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    return ROM_I2CMasterDataGet(I2C1_BASE);
}
