#include <math.h>
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
#include "logger.h"
#include "driverlib/pwm.h"
#include "gpsParser.h"
#include "altimeter.h"
#include "accelerometer.h"


/******************************************************************************************************/
/***************************************FUNCTION PROTOTYPES********************************************/
/******************************************************************************************************/

void Tiva_setup(void);
void timer_setup(void);
void timer_0A_handler(void);
void timer_1A_handler(void);
void UARTIntHandler(void);
void printStream(void);
void morse_tone_setup(void);



/******************************************************************************************************/
/*******************************************VARIABLES**************************************************/
/******************************************************************************************************/

//For GPS sensor
char gps_buffer[BUFFERSIZE];
char gps_dummy_buffer[750] = "$GPRMC,194426.00,A,4523.03374,N,07541.86434,W,0.281,,280315,,,A*61\r\n$GPVTG,,T,,M,0.281,N,0.521,K,A*2E\r\n$GPGGA,194426.00,9523.03374,S,17541.86434,E,1,06,1.71,30000.874,M,-34.2,M,,*67\r\n$GPGSA,A,3,03,16,31,23,29,,,,,,,,3.57,1.71,3.14*0E\r\n$GPGSV,3,1,11,03,31,251,15,08,71,036,23,14,06,137,08,16,56,212,24*7F\r\n$GPGSV,3,2,11,23,43,305,26,29,19,044,11,31,46,075,25,32,18,205,*75\r\n$GPGSV,3,3,11,46,35,211,,48,14,245,,51,29,221,*4B\r\n$GPGLL,4523.03374,N,07541.86434,W,194426.00,A,A*7E\r\n$GPRMC,194427.00,A,4523.03374,N,07541.86446,W,0.188,,280315,,,A*6F\r\n$GPVTG,,T,,M,0.188,N,0.348,K,A*2D\r\n$GPGGA,194427.00,4523.03374,N,07541.86446,W,1,05,1.71,147.6,M,-34.2,M,,*61\r\n$GPGSA,A,3,03,16,31,23,29,,,,,,,,3.57,1.71,3.14*0";
uint32_t index, count;
GPS_data gps_data;


//For Altimeter
uint16_t PROM_C[8];
Altimeter_data alti_data;  
uint32_t init_Pressure;

//For Accelerometer
int16_t acceletometer_data[3];


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

//************************************************************
//
//
//
//************************************************************
int main(void)
{

  Tiva_setup();
  // setup the logger
  
  logger_init();

  morse_tone_setup();
  UART_setup_gps();
  UART_setup_debug();
  UART_setup_transmitter();
  
  I2C1_setup();
  timer_setup();
  alti_setup();
  accel_setup();
 
  init_Pressure = alti_init_pres();

  
  while(1);
}




//**************************************************************
//
// The timer0A interrupt handler.
//
//**************************************************************
void timer_0A_handler(void)
{

  
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  parse_gps_data();
  alti_Convert();

  readAccelData();
  calcTiltAnglesFromAccelData();

  printStream();
  ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');
  ROM_UARTCharPutNonBlocking(UART0_BASE, '\n');

}


//**************************************************************
//
// The timer1A interrupt handler.
//
//**************************************************************
void timer_1A_handler(void)
{
  static char callsign[29] = {'.','.','.','-','P','.','-','P','.','.','.','-','-','P','.','-','P','.','-','P','-','.','.','-','P','-','.','.','.'};
  static uint8_t mc_idx = 0;
  static uint8_t mc_tick = 0;
  static uint8_t silence = 0;

  ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  if(mc_tick == 0){
     if(silence){
      ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
      silence = 0;
     }else {
      if(callsign[mc_idx] == '.'){
        ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
      }else if(callsign[mc_idx] == '-'){
        mc_tick = 2;
        ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
      }else{
        mc_tick = 2;
        ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
      }
      mc_idx++;
      if(mc_idx == 29){
        ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
        mc_idx = 0;
        mc_tick = 10;
      }
      silence = 1;
    }
  }else{
    mc_tick--;    
  }
}



//**************************************************************
//
// The UART interrupt handler.
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
    }
}


//************************************************************
//
// Initialization code for the protocols we use.
//
//************************************************************
void Tiva_setup(void)
{
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
  // Enable processor interrupts.
  //
  ROM_IntMasterEnable();
}



void timer_setup(void){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  //
  // Configure the two 32-bit periodic timers.
  //
  ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());

  //
  // Setup the interrupts for the timer timeouts.
  //
  ROM_IntEnable(INT_TIMER0A);
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  //
  // Enable the timers.
  //
  ROM_TimerEnable(TIMER0_BASE, TIMER_A);




  //Timer1A for Morsecode call sign, ticks every 500ms
   ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

  //
  // Configure the two 32-bit periodic timers.
  //
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()>>1);

  //
  // Setup the interrupts for the timer timeouts.
  //
  ROM_IntEnable(INT_TIMER1A);
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

  //
  // Enable the timers.
  //
  ROM_TimerEnable(TIMER1_BASE, TIMER_A);


}


void morse_tone_setup(void){
  // Uses M1PWM6  (Module 1, Generator 3, output 6) on PF2

  //Enable the GPIO F Pheriperal
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  // Set GPIO F2 to PMW function
  ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

  ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
  //Enable PWM module 1

  //Configure generator 3
  ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  //Set the period to be 512Hz tone, divide clock by 2^9
  ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 400);

  // Set the pulse width of output 6 for a 50% duty cycle.
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 200);

  
  //
  // Start the timers in generator 3.
  //
  ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  //
  // Enable the outputs.
  //ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

}



//*************************************************************
//
// Send GPS data over UART.
//
//*************************************************************

void printStream(void)
{
  char UART_Transmit_Data_GPS[54];
  char UART_Transmit_Data_ALTIMETER[27];
  char UART_Transmit_Data_ACCEL[15];

  int32_t Temp_temp, Pres_temp,Altitude_temp;


  //----------------------------GPS Data Conversion---------------------------------
  char *latitude = get_latitude();
  char *longitude = get_longitude();
  char *altitude = get_altitude();
  char *satellites = get_satellites_tracked();

  unsigned int i = 1;
  unsigned int j = 0;
  unsigned int k = 0;
  unsigned int o = 0;

  *(UART_Transmit_Data_GPS) = 'A';

  for (  ; i < 12 ; i++ ){
    *(UART_Transmit_Data_GPS+i) = *(latitude+i-1);
  }

  *(UART_Transmit_Data_GPS + i++) = ' ';
  *(UART_Transmit_Data_GPS + i++) = get_latitude_direction();
  //*(UART_Transmit_Data_GPS + i++) = ',';
  *(UART_Transmit_Data_GPS + i++) = ' ';

  *(UART_Transmit_Data_GPS + i++) = 'B';
  for (  ; j < 12 ; j++ ){
    *(UART_Transmit_Data_GPS+i+j) = *(longitude+j);
  }

  *(UART_Transmit_Data_GPS + i + j++) = ' ';
  *(UART_Transmit_Data_GPS + i + j++) = get_longitude_direction();
 // *(UART_Transmit_Data_GPS + i + j++) = ',';
  *(UART_Transmit_Data_GPS + i + j++) = ' ';

  *(UART_Transmit_Data_GPS + i + j++) = 'C';
  for (  ; k < 15 ; k++ ){
    *(UART_Transmit_Data_GPS+i+j+k) = *(altitude+k);
  }

  //*(UART_Transmit_Data_GPS+i+j+k++) = ',';
  *(UART_Transmit_Data_GPS+i+j+k++) = ' ';
  *(UART_Transmit_Data_GPS+i+j+k++) = 'D';
  for (  ; o < 2 ; o++ ){
    *(UART_Transmit_Data_GPS+i+j+k+o) = *(satellites+o);
  }

  *(UART_Transmit_Data_GPS + sizeof(UART_Transmit_Data_GPS) - 2) = '\r';
  *(UART_Transmit_Data_GPS + sizeof(UART_Transmit_Data_GPS) - 1) = '\n';


  //----------------------------END OF GPS Data Conversion---------------------------


  //---------------------------Altimeter Data Conversion-----------------------------
  Temp_temp = alti_data.Temp;
  Pres_temp = alti_data.Pres;
  Altitude_temp = alti_data.Altitude;

  UART_Transmit_Data_ALTIMETER[0] = 'E';
  if(Temp_temp<0){
    UART_Transmit_Data_ALTIMETER[1]='-';
    Temp_temp*=-1;
  }
  else{
    UART_Transmit_Data_ALTIMETER[1]='+';
  }

  UART_Transmit_Data_ALTIMETER[2] = Temp_temp/1000 + '0';
  Temp_temp%=1000;

  UART_Transmit_Data_ALTIMETER[3] = Temp_temp/100 + '0';
  Temp_temp%=100;

  UART_Transmit_Data_ALTIMETER[4] = '.';

  UART_Transmit_Data_ALTIMETER[5] = Temp_temp/10 + '0';
  Temp_temp%=10;

  UART_Transmit_Data_ALTIMETER[6] = Temp_temp + '0';

  UART_Transmit_Data_ALTIMETER[7] = ' ';

  //Pressure parsing
  UART_Transmit_Data_ALTIMETER[8] = 'F';

  if(Pres_temp<0){
    Pres_temp*=-1;
  }

  UART_Transmit_Data_ALTIMETER[9]   = Pres_temp/100000 + '0';
  Pres_temp%=100000;

  UART_Transmit_Data_ALTIMETER[10]  = Pres_temp/10000 + '0';
  Pres_temp%=10000;

  UART_Transmit_Data_ALTIMETER[11]  = Pres_temp/1000 + '0';
  Pres_temp%=1000;

  UART_Transmit_Data_ALTIMETER[12] = Pres_temp/100 + '0';
  Pres_temp%=100;

  UART_Transmit_Data_ALTIMETER[13] = '.';

  UART_Transmit_Data_ALTIMETER[14] = Pres_temp/10 + '0';
  Pres_temp%=10;

  UART_Transmit_Data_ALTIMETER[15] = Pres_temp + '0';


  //Altitude Calculations
  
  UART_Transmit_Data_ALTIMETER[16] = ' '; 
  UART_Transmit_Data_ALTIMETER[17] = 'G'; 

  UART_Transmit_Data_ALTIMETER[18]  =  Altitude_temp/100000 + '0';
  Altitude_temp%=100000;

  UART_Transmit_Data_ALTIMETER[19]  =  Altitude_temp/10000 + '0';
  Altitude_temp%=10000;

  UART_Transmit_Data_ALTIMETER[20]  =  Altitude_temp/1000 + '0';
  Altitude_temp%=1000;

  UART_Transmit_Data_ALTIMETER[21]  =  Altitude_temp/100 + '0';
  Altitude_temp%=100;

  UART_Transmit_Data_ALTIMETER[22]  =  '.';

  UART_Transmit_Data_ALTIMETER[23]  =  Altitude_temp/10 + '0';
  Altitude_temp%=10;

  UART_Transmit_Data_ALTIMETER[24]  =  Altitude_temp + '0';


  *(UART_Transmit_Data_ALTIMETER + sizeof(UART_Transmit_Data_ALTIMETER) - 2) = '\r';
  *(UART_Transmit_Data_ALTIMETER + sizeof(UART_Transmit_Data_ALTIMETER) - 1) = '\n';

  //----------------------------END OF Altimeter Data Conversion----------------------

  i=0;
  j=0;

   for (i = 0; i<3; i++) {

      if (i == 0) {
          UART_Transmit_Data_ACCEL[j++]='X';
      }
      else if (i == 1) {
          UART_Transmit_Data_ACCEL[j++]='Y';
      }
      else if (i == 2) {
          UART_Transmit_Data_ACCEL[j++]='Z';
      }

      if (acceletometer_data[i] < 0) {
          acceletometer_data[i] *= -1;
          UART_Transmit_Data_ACCEL[j++]='-';
      }
      else{
        UART_Transmit_Data_ACCEL[j++]='+';
      }

      UART_Transmit_Data_ACCEL[j++] = acceletometer_data[i]/10 + '0';
      UART_Transmit_Data_ACCEL[j++] = acceletometer_data[i]%10 + '0';

      UART_Transmit_Data_ACCEL[j++]=' ';
  }

  *(UART_Transmit_Data_ACCEL + sizeof(UART_Transmit_Data_ACCEL) - 1) = '\r';
  *(UART_Transmit_Data_ACCEL + sizeof(UART_Transmit_Data_ACCEL) - 1) = '\n';



  UARTSend((uint32_t) UART0_BASE, (uint8_t*) UART_Transmit_Data_GPS, sizeof(UART_Transmit_Data_GPS)/sizeof(char));

  UARTSend((uint32_t) UART5_BASE, (uint8_t*) UART_Transmit_Data_GPS, sizeof(UART_Transmit_Data_GPS)/sizeof(char));

  UARTSend((uint32_t) UART0_BASE, (uint8_t*) UART_Transmit_Data_ALTIMETER, sizeof(UART_Transmit_Data_ALTIMETER)/sizeof(char));
  
  UARTSend((uint32_t) UART5_BASE, (uint8_t*) UART_Transmit_Data_ALTIMETER, sizeof(UART_Transmit_Data_ALTIMETER)/sizeof(char));

  UARTSend((uint32_t) UART0_BASE, (uint8_t*) UART_Transmit_Data_ACCEL, sizeof(UART_Transmit_Data_ACCEL)/sizeof(char));
  
  UARTSend((uint32_t) UART5_BASE, (uint8_t*) UART_Transmit_Data_ACCEL, sizeof(UART_Transmit_Data_ACCEL)/sizeof(char));

  i=0;
  j=0;
  k=0;
  o=0;
}




