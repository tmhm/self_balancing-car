#ifndef __boma_key_H__
#define __boma_key_H__

#include "common.h"
#include "gpio.h"
#include "include.h"

//******用于拨码开关初始化***//

u8 my_key_valve;

void    boma_key_init(void);

void debug_boma_key();


#endif