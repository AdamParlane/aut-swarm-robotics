#ifndef OPT_INTERFACE_H_
#define OPT_INTERFACE_H_

#include "robotdefines.h"


#define resolution 0.00125

/******** Function Prototypes ********/
void SPI_Write(char writeAddress, char spiData);
char SPI_Read(char readAddress);
void SPI_Init(void);
void Mouse_Init(void);
int Mouse_Test(void);
void Get_Mouse_XY(struct Position *mousepPos);
void delay (void);

#endif /* OPT_INTERFACE_H_ */