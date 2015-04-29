#include "logger.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

void logger_init()
{
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  //
  ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
  ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  //
  // Configure the logger for 115200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
}

void logger_logString(char* ptr)
{
	if (ptr == 0)
		return;
	while(*ptr != 0){
	        ROM_UARTCharPutNonBlocking(UART4_BASE, *ptr++);
        while (ROM_UARTBusy(UART4_BASE));
    }

}




