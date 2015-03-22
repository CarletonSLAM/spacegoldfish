#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"

int main(void){

	//volatile to prevent GCC from optimizing out variable.
	volatile uint32_t ui32Loop;

	while (1);

}
