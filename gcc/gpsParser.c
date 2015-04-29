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
#include "driverlib/timer.h"

#include "gpsParser.h"


extern GPS_data gps_data;
extern char gps_dummy_buffer[750];
extern char gps_buffer[BUFFERSIZE];
extern uint32_t index, count;



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

//*************************************************************







//*************************************************************
//
// Send a string to the UART.
//
//*************************************************************
void UARTSend(uint32_t ui32Base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(ui32Base, *pui8Buffer++);
        while (ROM_UARTBusy(ui32Base));

    }
}

 
//*************************************************************
//
// Parse the GPS buffer and store information in the struct.
//
// You may notice this: (index%BUFFERSIZE) in multiple places.
// If our index exceeds our BUFFERSIZE then the modulus will
// allow us to wrap around and start at the beginning of our
// circular array.
//
//*************************************************************
void parse_gps_data(void)
{
  uint32_t index = 0;

  char *buffer = gps_buffer;

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


  // length 15 - altitude
  // length 10 - latitude
  // length 11 - longitude

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

  // insert a space in latitude and longitude strings.
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

// returns the index after the next ','
uint32_t _skip_data(char *buffer, uint32_t index)
{
  while (*(buffer+index) != ','){
    index = (index+1)%BUFFERSIZE;
  }
  return (index+1)%BUFFERSIZE;
}

// debugging stuff
/*void _sendIndex(uint32_t index)
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
}*/

/*
 *  Updates our gps coordinate structure with the newest buffer
 *  data.
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



/*
 *  Check for the nmea string(s) that will hold the
 *  information we need.
 */
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

void UART_setup_gps(void)
{
  //
  // Enable the peripherals used by gps.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  //
  // Set GPIO B0 and B1 as UART pins for the gps.
  //
  ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
  ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the GPS for 9600, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
  ROM_IntEnable(INT_UART1);
  ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void UART_setup_debug(void)
{
  //
  // Enable the peripherals used by the debug line.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Set GPIO A0 and A1 as UART pins for the debug.
  //
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
}

void UART_setup_transmitter(void){
    //
  // Enable the peripherals used by the debug line.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  //
  // Set GPIO A0 and A1 as UART pins for the debug.
  //
  ROM_GPIOPinConfigure(GPIO_PC4_U4RX);
  ROM_GPIOPinConfigure(GPIO_PC5_U4TX);
  ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), 57600,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

}
