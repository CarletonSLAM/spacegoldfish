
#ifndef __SPACEGOLDFISH_GPSPARSER_H__
#define __SPACEGOLDFISH_GPSPARSER_H__

#define BUFFERSIZE 450


typedef struct{
  char latitude[11];
  char north_south;

  char longitude[12];
  char east_west;

  char altitude[15];

  char number_of_satellites[2];

} GPS_data;


// Accessors for the GPS data.	
char *get_latitude(void);
char *get_longitude(void);
char *get_altitude(void);
char *get_satellites_tracked(void);
char get_latitude_direction(void);
char get_longitude_direction(void);


/***********************GPS FUNCTIONS***********************************/

//Init Functions
void UART_setup_gps(void);
void UART_setup_debug(void);
void UART_setup_transmitter(void);


//UART Send functions
void UARTSend(uint32_t ui32Base, const uint8_t *pui8Buffer, uint32_t ui32Count);

//Parse gps data functions
void parse_gps_data(void);
void parse_code(char *buffer, uint32_t index);
void update_coordinates(char *buffer, uint32_t index);
uint32_t set_location(char *buffer, uint32_t index, char *location, uint32_t length);
uint32_t set_satellites_tracked(char* buffer, char index);

/*************************************************************************/

#endif 		/* __SPACEGOLDFISH_GPSPARSER_H__ */
