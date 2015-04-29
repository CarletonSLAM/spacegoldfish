//#include <stdarg.h>
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
#include "driverlib/fpu.h"
#include <math.h>
#include "altimeter.h"

extern uint16_t PROM_C[8];
extern Altimeter_data alti_data;
extern int32_t init_Pressure;


void alti_setup(void){

  uint8_t char_array[16],i;
  
  //uint16_t PROM_C_dummy [8]={0x0840,0xAD8C, 0x9EE6, 0x6EDF, 0x65E4, 0x7C72, 0x6F00, 0xAF0B};


  alti_resetCMD();
  alti_promRead(char_array);
  for(i =0;i<8;i++){
    PROM_C[i]= (char_array[i*2]*2048)+char_array[i*2+1];
  }

}


void alti_Convert(void){


    uint32_t D1, D2;
    int32_t temp_dT, temp_T2,pres_Off2,pres_Sens2; 
    int64_t pres_Off, pres_Sens;

    temp_T2 = 0;
    pres_Off2 = 0;
    pres_Sens2 = 0;

    D1 = alti_I2CADCReceive((uint8_t)ALTI_D1_2048);

    D2 = alti_I2CADCReceive((uint8_t)ALTI_D2_2048);

    temp_dT = D2 - PROM_C[5] * pow(2,8);

    alti_data.Temp = 2000 + temp_dT*PROM_C[6]/pow(2,23);

    pres_Off = PROM_C[2]*pow(2,17)+(PROM_C[4]*temp_dT)/pow(2,6);
   
    pres_Sens = PROM_C[1]*pow(2,16) + (PROM_C[3]*temp_dT)/pow(2,7);
    
    if(alti_data.Temp< 2000){
      temp_T2 = pow(temp_dT,2)/pow(2,31);
      
      pres_Off2 = 61*pow((alti_data.Temp-2000),2)/16;
      
      pres_Sens2 = 2*pow((alti_data.Temp-2000),2);
      
      if(alti_data.Temp<-1500){
         pres_Off2 = pres_Off2  + 15*pow(alti_data.Temp +1500,2);
         pres_Sens2 =pres_Sens2 + 8*pow(alti_data.Temp +1500,2);
      }
    }


    //Store alti_data.Temp into Character form




    pres_Off -= pres_Off2;
    pres_Sens -= pres_Sens2;
    alti_data.Temp = alti_data.Temp - temp_T2;
    alti_data.Pres = (D1* pres_Sens/pow(2,21) - pres_Off) /pow(2,15);

    alti_data.Altitude = 443300* (1-pow(alti_data.Pres/init_Pressure,1/5.255));


}




void delay_uS(uint32_t us) {
  ROM_SysCtlDelay((ROM_SysCtlClockGet()/(3*1000000)*us ));  // more accurate
}



void I2C1_setup(void) {

  //Enable and reset I2C 1 channel
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

  //Enable the GPIO A Pheriperal
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Set GPIO A6 and A7 as I2C SCL and SDA pins
  ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
  ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

  ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
  ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);


  //Set the I2C clock bus speed to 100 kbps
  ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), false);

  //Clear the FIFO flags
  HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;


}

void alti_resetCMD(void) {

  //Load Device address to I2C_1 base with intended write
  ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS, false);

  //Load Reset Command to I2C_1 base
  ROM_I2CMasterDataPut(I2C1_BASE, ALTI_RESET);

  //Send I2C_1 base with a single send command
  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

  //wait till I2C_1 Bus is not busy
  while(ROM_I2CMasterBusy(I2C1_BASE));

  //Delay by around 800 uS
  delay_uS(800);

}



void alti_promRead( uint8_t *promBuffer) {
  uint8_t index;

  //---------------3 DUMMMY READS-----------------------------------
  for(index = 0;index<3;index++){
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS, false);
    ROM_I2CMasterDataPut(I2C1_BASE, ALTI_PROM_RD+(index*2));
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(ROM_I2CMasterBusy(I2C1_BASE));

    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS , true);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(ROM_I2CMasterBusy(I2C1_BASE));
  }
//---------------END OF 3 DUMMMY READS-----------------------------------

  //Loop done for the 128-bit PROM read
  for(index = 0;index<8;index++){

    //LOAD Device address to I2C_1 base with intended WRITE
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS, false);

    //LOAD the data that is going to be WRITTEN
    ROM_I2CMasterDataPut(I2C1_BASE, ALTI_PROM_RD+(index*2));  

    //SEND Data on I2C Bus with a Single send command     
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    //WAIT until transmission from master is complete
    while(ROM_I2CMasterBusy(I2C1_BASE));

    //LOAD Device address to I2C_1 base with intended READ
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS , true);

    //SEND Data on I2C Bus with Burst Receive Start Command
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    //WAIT until transmission from master is complete
    while(ROM_I2CMasterBusy(I2C1_BASE));

    //Store the MSB of 16-bit value in 1st cell of the 8-bit array
    *(promBuffer++) =(uint8_t) ROM_I2CMasterDataGet(I2C1_BASE);

    //SEND Data on I2C Bus with Burst Receive Finish Command
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //WAIT until transmission from master is complete
    while(ROM_I2CMasterBusy(I2C1_BASE));

    //Store the LSB of 16-bit value in 2nd cell of the 8-bit array
    *(promBuffer++) = (uint8_t) ROM_I2CMasterDataGet(I2C1_BASE);
  }

}


uint32_t alti_I2CADCReceive( uint8_t DataReg) {

  //Temporary variable
  uint32_t retData;


  ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS, false);

  ROM_I2CMasterDataPut(I2C1_BASE, DataReg);

  //SEND Data on I2C Bus with a Single Send Command
  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);


  //WAIT until transmission from master is complete
  while(ROM_I2CMasterBusy(I2C1_BASE));

  delay_uS(6000);

  ROM_I2CMasterDataPut(I2C1_BASE, 0x00);
  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

  //WAIT until transmission from master is complete
  while(ROM_I2CMasterBusy(I2C1_BASE));

 


  //read sequence
  ROM_I2CMasterSlaveAddrSet(I2C1_BASE, ALTI_ADDRESS, true);

  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
  while(ROM_I2CMasterBusy(I2C1_BASE));

  retData   = ROM_I2CMasterDataGet(I2C1_BASE);
  retData   = retData << 8;

  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(ROM_I2CMasterBusy(I2C1_BASE));

  retData  |= ROM_I2CMasterDataGet(I2C1_BASE);
  retData   = retData << 8;

  ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  while(ROM_I2CMasterBusy(I2C1_BASE));

  retData  |= ROM_I2CMasterDataGet(I2C1_BASE);

  return retData;

}


uint32_t alti_init_pres(void){

    uint32_t D1, D2;
    int32_t temp_dT, Temp_temp; 
    int64_t temp_T2, pres_Off, pres_Sens,pres_Off2,pres_Sens2;

    temp_T2 = 0;
    pres_Off2 = 0;
    pres_Sens2 = 0;

    D1 = alti_I2CADCReceive((uint8_t)ALTI_D1_2048);

    D2 = alti_I2CADCReceive((uint8_t)ALTI_D2_2048);

    temp_dT = D2 - PROM_C[5] * pow(2,8);
  
    Temp_temp = 2000 + temp_dT*PROM_C[6]/pow(2,23);

    
    if(Temp_temp< 2000){
      temp_T2 = pow(temp_dT,2)/pow(2,31);
      
      pres_Off2 = 61*pow((Temp_temp-2000),2)/16;
      
      pres_Sens2 = 2*pow((Temp_temp-2000),2);
      
      if(Temp_temp<-1500){
         pres_Off2 = pres_Off2  + 15*pow(Temp_temp +1500,2);
         pres_Sens2 =pres_Sens2 + 8*pow(Temp_temp +1500,2);
      }
    }



    pres_Off = PROM_C[2]*pow(2,17)+(PROM_C[4]*temp_dT)/pow(2,6);
   
    pres_Sens = PROM_C[1]*pow(2,16) + (PROM_C[3]*temp_dT)/pow(2,7);


    pres_Off -= pres_Off2;
    pres_Sens -= pres_Sens2;

    return (D1* pres_Sens/pow(2,21) - pres_Off) /pow(2,15);


}
