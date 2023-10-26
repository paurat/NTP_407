/*
 * MyFlash.h
 *
 *  Created on: 18 мар. 2022 г.
 *      Author: User
 */

#ifndef INC_MYFLASH_H_
#define INC_MYFLASH_H_
#include "stdio.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"
void clearFlash();
void WriteDeviceAddressOffset(char* data, int size, int offset);
void ReadDeviceAddressOffset(char* Dout, int size, int offset);

#endif /* INC_MYFLASH_H_ */
