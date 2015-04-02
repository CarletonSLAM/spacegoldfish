//#include <stdlib.h>
#include <malloc.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void GPSSend(void);

uint32_t* get_latitude(void);
uint32_t* get_longitude(void);
uint32_t* get_latitude_direction(void);
uint32_t* get_longitude_direction(void);

uint32_t gps_buffer[700];
uint32_t index, count;

typedef struct{
  uint32_t latitude[8];
  uint32_t longitude[8];
  uint32_t north_south;
  uint32_t east_west;
} GPS_data;

GPS_data gps_data;


//**************************************************************
//
// Accessors for the GPS data.
//
//**************************************************************

uint32_t* get_latitude(void)
{
  return gps_data.latitude;
}

uint32_t* get_longitude(void)
{
  return gps_data.longitude;
}

uint32_t* get_latitude_direction(void)
{
  return &gps_data.north_south;
}

uint32_t* get_longitude_direction(void)
{
  return &gps_data.east_west;
}



//**************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//**************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//**************************************************************
//
// The UART interrupt handler.
//
//**************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t c;
    uint32_t counter = 0;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while (ROM_UARTCharsAvail(UART1_BASE))
    {
      counter++;
        //
        // Read the next character from the UART and write it back to the UART.
        //
        c = ROM_UARTCharGetNonBlocking(UART1_BASE);

        if (index++ > 699){
          index = 0;
        }

        gps_buffer[index] = c;

        //
        // Blink the LED to show a character transfer is occuring.
        //
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        ROM_SysCtlDelay(ROM_SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }

    //UARTSend((uint8_t*) counter, 1);
}

//*************************************************************
//
// Send a string to the UART.
//
//*************************************************************
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
    }
}

//*************************************************************
//
// Send GPS data over UART.
//
//*************************************************************

void GPSSend(void)
{
  uint32_t latitude[10];
  uint32_t longitude[10];

  for (int i = 0 ; i < 8 ; i++){
    longitude[i] = gps_data.longitude[i];
    latitude[i] = gps_data.latitude[i];
  }
  longitude[8] = ' ';
  longitude[9] = gps_data.east_west;

  latitude[8] = ' ';
  latitude[9] = gps_data.north_south;

  UARTSend((uint8_t*) latitude, 10);
  UARTSend((uint8_t*) longitude, 10);

}

//*************************************************************
//
// Parse the GPS buffer and store information in the struct.
//
//*************************************************************


//************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//************************************************************
int main(void)
{
    uint32_t count;
    index = 0;
    count = 0;

    //gps_data = malloc(sizeof(GPS_data));

    gps_data.latitude[0] = '0';
    gps_data.latitude[1] = '7';
    gps_data.latitude[2] = '5';
    gps_data.latitude[3] = '.';
    gps_data.latitude[4] = '6';
    gps_data.latitude[5] = '9';
    gps_data.latitude[6] = '7';
    gps_data.latitude[7] = '6';

    gps_data.longitude[0] = '4';
    gps_data.longitude[1] = '5';
    gps_data.longitude[2] = '.';
    gps_data.longitude[3] = '3';
    gps_data.longitude[4] = '8';
    gps_data.longitude[5] = '3';
    gps_data.longitude[6] = '1';
    gps_data.longitude[7] = ' ';

    gps_data.north_south = 'N';
    gps_data.east_west = 'E';

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    while(1){
      count++;
      if(count > 200000){
        count = 0;
        ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');
        ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');
        //UARTSend((uint8_t*) gps_buffer, 699);
        GPSSend();
      }
    }

}
