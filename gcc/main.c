//#include <stdlib.h>
//#include <malloc.h>
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
//**************************************************************
//    ADD DESCRIPTIONSSS
//    Note:You may notice this: (index%BUFFERSIZE) in multiple places. If our
//    index exceeds our BUFFERSIZE then the modulus will allow us to wrap around
//    and start at the beginning of our circular array.
//**************************************************************

//  FUNCTION PROTOTYPES
//**************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void GPSSend(void);
void parse_gps_data(void);
void parse_code(char *buffer, uint32_t index);
void update_coordinates(char *buffer, uint32_t index);
void initConfig(void);
void initDebugTerminal(void);

char *get_latitude(void);
char *get_longitude(void);
char *get_altitude(void);
char *get_satellites_tracked(void);
char get_latitude_direction(void);
char get_longitude_direction(void);

uint32_t set_location(char *buffer, uint32_t index, char *location, uint32_t length);
uint32_t set_satellites_tracked(char* buffer, char index);
//**************************************************************


//  CONSTANTS           
//**************************************************************
#define BUFFERSIZE 450
//**************************************************************


//  STRUCTURE DECLERATIONS
//**************************************************************
typedef struct{
  char latitude[11];
  char north_south;

  char longitude[12];
  char east_west;

  char altitude[15];

  char number_of_satellites[2];

} GPS_data;
//**************************************************************



//  VARIABLES
//**************************************************************
char gps_buffer[BUFFERSIZE];
char gps_dummy_buffer[750] = "$GPRMC,194426.00,A,4523.03374,N,07541.86434,W,0.281,,280315,,,A*61\r\n$GPVTG,,T,,M,0.281,N,0.521,K,A*2E\r\n$GPGGA,194426.00,9523.03374,S,17541.86434,E,1,06,1.71,30000.874,M,-34.2,M,,*67\r\n$GPGSA,A,3,03,16,31,23,29,,,,,,,,3.57,1.71,3.14*0E\r\n$GPGSV,3,1,11,03,31,251,15,08,71,036,23,14,06,137,08,16,56,212,24*7F\r\n$GPGSV,3,2,11,23,43,305,26,29,19,044,11,31,46,075,25,32,18,205,*75\r\n$GPGSV,3,3,11,46,35,211,,48,14,245,,51,29,221,*4B\r\n$GPGLL,4523.03374,N,07541.86434,W,194426.00,A,A*7E\r\n$GPRMC,194427.00,A,4523.03374,N,07541.86446,W,0.188,,280315,,,A*6F\r\n$GPVTG,,T,,M,0.188,N,0.348,K,A*2D\r\n$GPGGA,194427.00,4523.03374,N,07541.86446,W,1,05,1.71,147.6,M,-34.2,M,,*61\r\n$GPGSA,A,3,03,16,31,23,29,,,,,,,,3.57,1.71,3.14*0";
//char gps_dummy_buffer[550] = "$GPRMC,194426.00,A,4523.03374,N,07541.86434,W,0.281,,280315,,,A*61\r\n$GPVTG,,T,,M,0.281,N,0.521,K,A*2E\r\n$GPGSA,A,3,03,16,31,23,29,,,,,,,,3.57,1.71,3.14*0E\r\n$GPGSV,3,1,11,03,31,251,15,08,71,036,23,14,06,137,08,16,56,212,24*7F\r\n$GPGSV,3,2,11,23,43,305,26,29,19,044,11,31,46,075,25,32,18,205,*75\r\n$GPGSV,3,3,11,46,35,211,,48,14,245,,51,29,221,*4B\r\n$GPGLL,4523.03374,N,07541.86434,W,194426.00,A,A*7E\r\n";
//char gps_dummy_buffer[750] = "$GPGGA,194426.00,4523.03374,N,07541.86434,W,1,05,1.71,147.9084,M,-34.2,M,,*67\r\n";
uint32_t index, count;
GPS_data gps_data;
//**************************************************************


//**************************************************************
//
// Accessors for the GPS data.
//
//**************************************************************

char* get_latitude(void)
{
  return gps_data.latitude;
}

char* get_longitude(void)
{
  return gps_data.longitude;
}

char* get_altitude(void)
{
  return gps_data.altitude;
}

char* get_satellites_tracked(void)
{
  return gps_data.number_of_satellites;
}

char get_latitude_direction(void)
{
  return gps_data.north_south;
}

char get_longitude_direction(void)
{
  return gps_data.east_west;
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
//  Function: UARTIntHandler
//
//    The UART interrupt handler.
//
//**************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;
    char c;
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

        if (index++ >= BUFFERSIZE){
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
// Function: UARTSend
//    Sends a string to the UART2 and UART0.
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
        
        while (ROM_UARTBusy(UART0_BASE));
        
        //ROM_UARTCharPutNonBlocking(UART2_BASE, *pui8Buffer++);
        //while (ROM_UARTBusy(UART2_BASE));
    }
}

//*************************************************************
//
// Send GPS data over UART.
//
//*************************************************************

void GPSSend(void)
{
  char location[52];
  char *latitude = get_latitude();
  char *longitude = get_longitude();
  char *altitude = get_altitude();
  char *satellites = get_satellites_tracked();

  unsigned int i = 0;
  unsigned int j = 0;
  unsigned int k = 0;
  unsigned int o = 0;

  for (  ; i < 11 ; i++ ){
    *(location+i) = *(latitude+i);
  }

  *(location + i++) = ' ';
  *(location + i++) = get_latitude_direction();
  *(location + i++) = ',';
  *(location + i++) = ' ';


  for (  ; j < 12 ; j++ ){
    *(location+i+j) = *(longitude+j);
  }

  *(location + i + j++) = ' ';
  *(location + i + j++) = get_longitude_direction();
  *(location + i + j++) = ',';
  *(location + i + j++) = ' ';

  for (  ; k < 15 ; k++ ){
    *(location+i+j+k) = *(altitude+k);
  }


  *(location+i+j+k++) = ',';
  *(location+i+j+k++) = ' ';

  for (  ; o < 2 ; o++ ){
    *(location+i+j+k+o) = *(satellites+o);
  }


  *(location + sizeof(location) - 2) = '\r';
  *(location + sizeof(location) - 1) = '\n';


  UARTSend((uint8_t*) location, sizeof(location));

}

//*************************************************************
//
// Parse the GPS buffer and store information in the struct.
//
//*************************************************************
void parse_gps_data(void)
{
  uint32_t index = 0;

  char *buffer = gps_dummy_buffer;

  while (index <= BUFFERSIZE){

    if (*(buffer + index) != '$'){
      index++;
      continue;
    }
    parse_code(buffer, ++index);
//    break;
  }
}

uint32_t set_location(char *buffer, uint32_t index, char *location, uint32_t length)
{
  uint32_t i;
  // when incrementing we'll need to subtract the index offset
  uint32_t offset = 0;

  index = index%BUFFERSIZE;

  // the nmea strings from the gps is comma delimited so loop till you find one
  while (*(buffer+index) != ','){
    *(location+offset) = *(buffer+index);
    // remember to wrap around.
    index = (index+1)%BUFFERSIZE;
    offset++;
  }
  // skip the comma
  index = (index+1)%BUFFERSIZE;

  // grab the direction
  if (length == 15){
    // the case for the altitude is different.
    *(location+offset) = ' ';
    offset++;
    *(location+offset) = *(buffer+index);
    while (++offset < 15){
      *(location+offset) = ' ';
    }
    return -1;
  }else if (length == 10){
    gps_data.north_south = *(buffer+index);
  }else{
    gps_data.east_west = *(buffer+index);
  }
  // skip comma
  index = (index+1)%BUFFERSIZE;
  index = (index+1)%BUFFERSIZE;


  for (i = length; i > length - 8; i--){
    *(location+i) = *(location+i-1);
  }
  *(location+i) = ' ';

  return index;

}

uint32_t set_satellites_tracked(char *buffer, char index)
{
  // when increamenting we'll need to subtract the index offset
  uint32_t offset = 0;
  char *satellites = get_satellites_tracked();

  // index = index%BUFFERSIZE;

  // the nmea strings from the gps is comma delimited so loop till you find one
  while (*(buffer+index) != ','){
    *(satellites+offset) = *(buffer+index);
    // remember to wrap around.
    index = (index+1)%BUFFERSIZE;
    offset++;
  }

  return (index+1)%BUFFERSIZE;
}

uint32_t _skip_data(char *buffer, uint32_t index)
{
  while (*(buffer+index) != ','){
    index = (index+1)%BUFFERSIZE;
  }
  return (index+1)%BUFFERSIZE;


}

void sendIndex(uint32_t index)
{
  uint32_t tmp = index/10;
  char num[16] = "            \n";
  char i = 0;
  index = index%10;
  while (index != 0){
    num[8 - i++] = (char) index + '0';
    index = tmp%10;
    tmp = tmp/10;
  }
  UARTSend((uint8_t*)num, 16);
}

/*
 *  You may notice this: (index%BUFFERSIZE) in multiple places. If our
 *  index exceeds our BUFFERSIZE then the modulus will allow us to wrap around
 *  and start at the beginning of our circular array.
 */
void update_coordinates(char *buffer, uint32_t index)
{
  char *latitude = get_latitude();
  char *longitude = get_longitude();
  char *altitude = get_altitude();

  index = index%BUFFERSIZE;

  // skip the time
  index = _skip_data(buffer, index);

  index = set_location(buffer, index, latitude, 10);
  index = set_location(buffer, index, longitude, 11);

  // skip the fix
  index = _skip_data(buffer, index);
  //sendIndex(index);

  //index = (index+2)%BUFFERSIZE;
  // get satalites tracked
  index = set_satellites_tracked(buffer, index);
  //sendIndex(index);

  //sendIndex(index);
  // skip Horizontal dilution of position
  index = _skip_data(buffer, index);

  // altitude
  index = set_location(buffer, index, altitude, 15);

}



//----------------------------------------------------------------------------------
//  Function: parse_code
//
//      Finds starting code for a GPGGA section in the NMEA code in GPS_BUFFER
//----------------------------------------------------------------------------------
void parse_code(char *buffer, uint32_t index)
{
  //char *gpgll = "GPGGA";
  //char *gpgsv = "GPGSV";

  // All pieces of data start with GP
  if (*(buffer+(index%BUFFERSIZE)) != 'G') return;
  index++;
  if (*(buffer+(index%BUFFERSIZE)) != 'P') return;
  index++;

  // Check for the GLL data
  if (*(buffer+(index%BUFFERSIZE))=='G'
    && *(buffer+((index+1)%BUFFERSIZE))=='G'
    && *(buffer+((index+2)%BUFFERSIZE))=='A')
  {
    update_coordinates(buffer, (index+4)%BUFFERSIZE);
  }

}


//----------------------------------------------------------------------------------
// Function: initConfig
//
//   Initializations for configurations needed
//----------------------------------------------------------------------------------
void initConfig(void){

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable the GPIO port that is used for the on-board LED. (FOR VISUAL FEEDBACK)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF2).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Enable processor interrupts.
    ROM_IntMasterEnable();

}

//----------------------------------------------------------------------------------
// Function: initDebugTerminal
//
//  Initializes UART0 to use as a debug console in PC
//  
//  to see messages:
//      in Linix:   sudo miniterm.py /dev/"Device ID" "baudRate"
//      in Windows: Download putty
//                  Open serial channel on assigned COM port at the given Baud Rate       
//----------------------------------------------------------------------------------
void initDebugTerminal(void){

  // Initialize UART0 and GPIOA functions for Tiva C
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //Configure Pin PA0 as UART0-RX and Pin PA1 as UART0-TX
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  
  //Configure Pins 1 and 2 in Port A to be served by UART0
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the UART1 for 8-N-1 operation with a Buad Rate of 115200
  ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}



//----------------------------------------------------------------------------------
//
//  Function: main
//
//    Actively polls the GPS module and transmits it thorough the Transmitter in UART2
//----------------------------------------------------------------------------------
int main(void){

//----------------------------------------------------------------------------------
  uint32_t count;
    index = 0;
    count = 0;

    initConfig();

    initDebugTerminal();
    
    //-------------Initializations for GPS Module-----------------------------------

    //  Initialize UART1 and GPIOB functions for GPS Module
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
   
    //Configure Pin PB0 as UART1-RX and Pin PB1 as UART1-TX
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);

    //Configure Pins 1 and 2 in Port B to be served by UART1
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART1 for 8-N-1 operation with a Buad Rate of 9600
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


    //-------------Initializations for Digital Tranmitter Module----------------------
    
    //  Initialize UART2 and GPIOD functions for Digital Transmitter
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //Configure Pin PD6 as UART2-RX and Pin PD7 as UART2-TX
    ROM_GPIOPinConfigure(GPIO_PD6_U2RX);
    ROM_GPIOPinConfigure(GPIO_PD7_U2TX);

    //Configure Pins 6 and 7 in Port D to be served by UART2
    //ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Configure the UART1 for 8-N-1 operation with a Buad Rate of 57600
    //ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    

    // Enable the UART interrupt.
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    while(1){
      count++;
      if(count > 1500000){
        count = 0;
        ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');
        //ROM_UARTCharPutNonBlocking(UART2_BASE, '\n');

        parse_gps_data();
        GPSSend();  
      }
    }

}

