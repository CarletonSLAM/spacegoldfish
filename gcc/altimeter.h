
#ifndef __SPACEGOLDFISH_ALTIMETER_H__
#define __SPACEGOLDFISH_ALTIMETER_H__



#define ALTI_ADDRESS      0x77    
#define ALTI_RESET        0x1E      // ADC reset command 
#define ALTI_PROM_RD      0xA0
#define ALTI_D1_2048      0x46
#define ALTI_D2_2048      0x56

typedef struct{

  int32_t Temp;
  int32_t Pres;
  int32_t Altitude;

} Altimeter_data;

/***********************ALTIMETER Functions***********************************/

void I2C1_setup(void);
void alti_setup(void);
void alti_resetCMD(void);
void alti_promRead(uint8_t *);
void alti_Convert(void);
uint32_t alti_I2CADCReceive (uint8_t);
void delay_uS(uint32_t);
uint32_t alti_init_pres(void);

/*************************************************************************/


#endif		/*__SPACEGOLDFISH_ALTIMETER_H__*/